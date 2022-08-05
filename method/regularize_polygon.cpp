//
// Created by Jin on 19/07/2022.
//
#include "regularize_polygon.h"
#include "dbscan.h"
#include <gurobi_c++.h> //use gurobi as the solver


std::vector<vec2> compute_all_directions(const std::vector<vec2> &polygon)
{
    std::vector<vec2> directions;
    int num_points = polygon.size();

    for (int i = 0; i < num_points; ++i)
    {
        const std::size_t ip = (i + 1) % num_points;
        auto dir = normalize((polygon[ip] - polygon[i]));
        auto orthognal_dir = vec2(-dir.y, dir.x);
        directions.push_back(dir);
        directions.push_back(orthognal_dir);

    }
    for (int i = 0; i < directions.size(); ++i)
    {
        auto dir = directions[i];
        if (dir.y < 0)
        {
            directions[i] = -dir;
        }
    }

    return directions;
};

float dist_func_2(const vec2 &a, const vec2 &b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std::vector<vec2> cluster_dirs(std::vector<vec2> all_dircs)
{
    DBSCAN<vec2, float> dbscan;
    dbscan.Run(&all_dircs, 2, 0.15f, 2, dist_func_2);

    auto noise = dbscan.Noise;
    auto clusters = dbscan.Clusters;
    std::vector<vec2> principal_dircs;

    for (int ind = 0; ind < clusters.size(); ++ind)
    {
        auto cluster = clusters[ind];
        vec2 average(0, 0);
        for (int i = 0; i < cluster.size(); ++i)
        {
            average += all_dircs[cluster[i]];
        }
        average /= cluster.size();
        average = normalize(average);
        principal_dircs.push_back(average);
        for (int i = 0; i < cluster.size(); ++i)
        {
            all_dircs[cluster[i]] = average;
        }
    }
    return principal_dircs;
}

std::vector<vec2> RegularizePolygon::reg_ply(const std::vector<vec2> &polygon)
{

    //get all directions of polygon
    auto all_dircs = compute_all_directions(polygon);

    //cluster all directions using dbscan.h
    auto principal_dircs = cluster_dirs(all_dircs);
    //enhance orthogonality of directions
    principal_dircs = orthognal_dircs(principal_dircs);
    std::vector<double> reg_polygon_slope;
    reg_polygon_slope.resize(polygon.size());

    for (int i = 0; i < polygon.size(); i++)
    {
        int in = i, ip = (i + 1) % polygon.size();
        auto s = polygon[in];
        auto t = polygon[ip];
        vec2 st_dir = normalize(s - t);
        //find the closest direction in principal directions
        double theta = 180;
        vec2 target_dir(st_dir);
        for (auto pri_dir: principal_dircs)
        {
            double angle = acos(std::abs(dot(st_dir, pri_dir) / (length(st_dir) * length(pri_dir)))) * 57.3;
            if (angle < theta)
            {
                theta = angle;
                target_dir = pri_dir;
            }
        }
        if (theta < 20)
        {
            reg_polygon_slope[i] = target_dir.x / (target_dir.y + 1e-5);//avoid divide by zero
        } else
        {
            reg_polygon_slope[i] = st_dir.x / (st_dir.y + 1e-5);//avoid divide by zero
        }
    }

    std::vector<vec2> reg_polygon = optimize_polygon(polygon, reg_polygon_slope);


    return reg_polygon;
}

std::vector<vec2> RegularizePolygon::orthognal_dircs(std::vector<vec2> vector1)
{
    std::vector<bool> check_oriented(vector1.size(), false);
    for (int i = 0; i < vector1.size(); ++i)
    {
        auto source_dir = vector1[i];
        for (int j = i; j < vector1.size(); ++j)
        {
            if (!check_oriented[j])
            {
                auto target_dir = vector1[j];
                double angle = acos(std::abs(dot(source_dir, target_dir))) * 57.3;
                if (angle > 80)
                {
                    check_oriented[j] = true;
                    vector1[j] = vec2(-source_dir.y, source_dir.x);
                }
            }

        }
    }

    return vector1;
}

std::vector<vec2> RegularizePolygon::optimize_polygon(const std::vector<vec2> &polygon, std::vector<double> slope)
{
    try
    {
        static GRBEnv env = GRBEnv();
        env.set(GRB_IntParam_LogToConsole, 0);
        std::vector<double> pts;
        for (auto pt: polygon)
        {
            pts.push_back(pt.x);
            pts.push_back(pt.y);
        }
        auto max_val = *std::max_element(std::begin(pts), std::end(pts));
        auto min_val = *std::min_element(std::begin(pts), std::end(pts));
        auto lb = 2 * min_val - max_val, ub = 2 * max_val - min_val;

        GRBModel model = GRBModel(env);

        // create variables, the num of variables equals to 2*num_edge
        std::vector<GRBVar> X(polygon.size() * 2);

        //add continuous variable
        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            X[i] = model.addVar(lb, ub, 0.0, GRB_CONTINUOUS);
        }
        // Integrate new variables
        model.update();

        for (int ind = 0; ind < polygon.size(); ind++)
        {
            unsigned int in = ind, ip = (ind + 1) % polygon.size();
            auto vsx1 = X[in * 2], vsy1 = X[in * 2 + 1];
            auto vtx1 = X[ip * 2], vty1 = X[ip * 2 + 1];
            double slope1 = slope[in];
            model.addConstr(vtx1 - vsx1 == slope1 * (vty1 - vsy1));
        }

        // Set objective
        GRBQuadExpr obj;
        double mu = 1.;

        for (int j = 0; j < pts.size(); ++j)
        {
            obj += (X[j] - pts[j]) * (X[j] - pts[j]);
        }
        for (int i = 0; i < polygon.size(); ++i)
        {
            int in = i, ip = (i + 1) % polygon.size();
            auto vsx1 = X[in * 2], vsy1 = X[in * 2 + 1];
            auto vtx1 = X[ip * 2], vty1 = X[ip * 2 + 1];
            obj+= mu*((vtx1 - vsx1) * (vtx1 - vsx1) + (vty1 - vsy1) * (vty1 - vsy1));
        }

        // Set objective function sense
        model.setObjective(obj, GRB_MINIMIZE);

        // Optimize
        model.optimize();
        int status = model.get(GRB_IntAttr_Status);
        auto obj_val = obj.getValue();
        switch (status)
        {
            case GRB_OPTIMAL:
            {
                std::vector<vec2> reg_polygon;
                for (int i = 0; i < polygon.size(); ++i)
                {
                    reg_polygon.push_back(vec2(X[i * 2].get(GRB_DoubleAttr_X), X[i * 2 + 1].get(GRB_DoubleAttr_X)));
                }
                return reg_polygon;
            }

            case GRB_INF_OR_UNBD:
                std::cerr << "model is infeasible or unbounded" << std::endl;
                break;

            case GRB_INFEASIBLE:
                std::cerr << "model is infeasible" << std::endl;
                break;

            case GRB_UNBOUNDED:
                std::cerr << "model is unbounded" << std::endl;
                break;

            default:
                std::cerr << "optimization was stopped with status = " << status << std::endl;
                break;
        }
    }
    catch (GRBException e)
    {
        std::cout << e.getMessage() << " (error code: " << e.getErrorCode() << ")." << std::endl;
        if (e.getErrorCode() == GRB_ERROR_NO_LICENSE)
        {
            std::cout
                    << "Gurobi installed but license is missing or expired. Please choose another solver, e.g., SCIP."
                    << std::endl;
        }
    }
    catch (...)
    {
        std::cerr << "Exception during optimization" << std::endl;
    }


}
