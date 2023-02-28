//
// Created by Jin on 19/07/2022.
//
#include "regularize_polygon.h"
#include "dbscan.h"

#ifdef  HAS_GUROBI
#include <gurobi_c++.h> //use gurobi as the solver
#else

#include "scip/scip.h" //use scip as the solver
#include "scip/scipdefplugins.h"

#endif

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
    std::vector<vec2> plys = polygon;

    optimize_polygon(plys, reg_polygon_slope);

    return plys;
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

bool RegularizePolygon::optimize_polygon(std::vector<vec2> &polygon, std::vector<double> slope)
{
#ifdef HAS_GUROBI
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
        std::vector<double> edge_length;
        for (int i = 0; i < polygon.size(); ++i)
        {
            int in = i, ip = (i + 1) % polygon.size();
            auto s = polygon[in];
            auto t = polygon[ip];
            edge_length.push_back(length2(s - t));
        }

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
            auto len = edge_length[i];
            //obj+= mu*((vtx1 - vsx1) * (vtx1 - vsx1) + (vty1 - vsy1) * (vty1 - vsy1)-len);
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
                polygon=reg_polygon;
                return true;
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
#else
    try
    {
        Scip *scip = 0;
        SCIP_CALL(SCIPcreate(&scip));
        SCIP_CALL(SCIPincludeDefaultPlugins(scip));
        // disable scip output to stdout
        SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(scip), TRUE);


        // use wall clock time because getting CPU user seconds
        // involves calling times() which is very expensive
        SCIP_CALL(SCIPsetIntParam(scip, "timing/clocktype", SCIP_CLOCKTYPE_WALL));

        // create empty problem
        SCIP_CALL(SCIPcreateProbBasic(scip, "City3D"));

        SCIP_CALL(SCIPfreeTransform(scip));
        SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

        // create variables
        std::vector<double> pts;
        for (auto pt: polygon)
        {
            pts.push_back(pt.x);
            pts.push_back(pt.y);
        }
        auto max_val = *std::max_element(std::begin(pts), std::end(pts));
        auto min_val = *std::min_element(std::begin(pts), std::end(pts));
        auto lb = 2 * min_val - max_val, ub = 2 * max_val - min_val;
        std::vector<double> edge_length;
        for (int i = 0; i < polygon.size(); ++i)
        {
            int in = i, ip = (i + 1) % polygon.size();
            auto s = polygon[in];
            auto t = polygon[ip];
            edge_length.push_back(length2(s - t));
        }
        std::vector<SCIP_VAR *> scip_var_x;
        std::vector<SCIP_VAR *> scip_var_t;
        std::vector<SCIP_VAR *> scip_var_m;

        std::vector<SCIP_Real> coef;
        coef.push_back(1.0);
        coef.push_back(-1.0);

        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            const std::string &namex = "x" + std::to_string(i + 1);
            const std::string &namet = "t" + std::to_string(i + 1);
            const std::string &namem = "m" + std::to_string(i + 1);
            SCIP_VAR *x = 0;
            SCIP_VAR *t = 0;
            SCIP_VAR *m = 0;
            SCIP_CALL(SCIPfreeTransform(scip));
            SCIP_CALL(SCIPcreateVarBasic(scip, &x, namex.data(), lb, ub, 0., SCIP_VARTYPE_CONTINUOUS));
            SCIP_CALL(SCIPcreateVarBasic(scip, &t, namet.data(), 0, SCIPinfinity(scip), 1, SCIP_VARTYPE_CONTINUOUS));
          //  SCIP_CALL(SCIPcreateVarBasic(scip, &m, namem.data(), -10, SCIPinfinity(scip), 1, SCIP_VARTYPE_CONTINUOUS));
            // add the SCIP_VAR object to the scip problem
            SCIP_CALL(SCIPaddVar(scip, x));
            SCIP_CALL(SCIPaddVar(scip, t));
           // SCIP_CALL(SCIPaddVar(scip, m));
            // storing the SCIP_VAR pointer for later access
            scip_var_x.push_back(x);
            scip_var_t.push_back(t);
          //  scip_var_m.push_back(m);
        }

        //add linear constraints
        for (std::size_t i = 0; i < slope.size(); ++i)
        {
            unsigned int in = i, ip = (i + 1) % polygon.size();
            SCIP_Var *vrs[4];
            SCIP_Real coefs[4];
            double slope1 = slope[in];
            vrs[0] = scip_var_x[in * 2];
            coefs[0] = -1;
            vrs[1] = scip_var_x[in * 2 + 1];
            coefs[1] = slope1;
            vrs[2] = scip_var_x[ip * 2];
            coefs[2] = 1;
            vrs[3] = scip_var_x[ip * 2 + 1];
            coefs[3] = -slope1;
            SCIP_CONS *cons = 0;
            const std::string &name = "cstr" + std::to_string(i + 1);
            SCIP_CALL(SCIPfreeTransform(scip));
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name.data(), 4, vrs, coefs, 0., 0.));
            SCIP_CALL(SCIPaddCons(scip, cons));            // add the constraint to scip
        }


        // Add constraints

        for (int i = 0; i < pts.size(); ++i)
        {
            SCIP_CONS *cons;
            SCIP_EXPR *xexpr;
            SCIP_Expr *expr;
            SCIP_EXPR *expr1;
            SCIP_Expr *powexpr;
            SCIP_EXPR *expvrs[2];
            SCIP_Real coefs[2];
            /* create variable expression */
            SCIP_CALL(SCIPcreateExprVar(scip, &xexpr, scip_var_x[i], NULL, NULL));
            SCIP_CALL(SCIPcreateExprSum(scip, &expr, 1, &xexpr, NULL, SCIP_Real(-pts[i]), NULL, NULL));
            SCIP_CALL(SCIPcreateExprPow(scip, &powexpr, expr, 2.0, NULL, NULL));
            SCIP_CALL(SCIPcreateExprVar(scip, &expvrs[0], scip_var_t[i], NULL, NULL));
            coefs[0] = 1.0;
            expvrs[1] = powexpr;
            coefs[1] = -1.0;
            SCIP_CALL(SCIPcreateExprSum(scip, &expr1, 2, expvrs, coefs, 0.0, NULL, NULL));
            SCIP_CALL(SCIPcreateConsBasicNonlinear(scip, &cons, "consname", expr1, 0.0, 0.));
            SCIP_CALL(SCIPaddCons(scip, cons));
        }

        if (0)
        {
            for (int i = 0; i < polygon.size(); ++i)
            {
//              obj+= mu*((vtx1 - vsx1) * (vtx1 - vsx1) + (vty1 - vsy1) * (vty1 - vsy1)-len)^2;
                int in = i, ip = (i + 1) % polygon.size();
                auto vsx = scip_var_x[in * 2], vsy = scip_var_x[in * 2 + 1];
                auto vtx = scip_var_x[ip * 2], vty = scip_var_x[ip * 2 + 1];
                auto len2 = edge_length[i];
                SCIP_CONS *cons1;
                SCIP_Real coefs[2];
                coefs[0] = 1.0;
                coefs[1] = -1.0;
                SCIP_EXPR *xexpr[2];
                SCIP_EXPR *yexpr[2];
                SCIP_EXPR *expr1;
                SCIP_EXPR *expr2;
                SCIP_EXPR *powexpr1;
                SCIP_EXPR *powexpr2;
                SCIP_EXPR *sumexpr[2];
                SCIP_EXPR *lastpvrs;
                SCIP_EXPR *lastpvrs1;
                SCIP_CALL(SCIPcreateExprVar(scip, &xexpr[0], vsx, NULL, NULL));
                SCIP_CALL(SCIPcreateExprVar(scip, &xexpr[1], vtx, NULL, NULL));
                SCIP_CALL(SCIPcreateExprVar(scip, &yexpr[0], vsy, NULL, NULL));
                SCIP_CALL(SCIPcreateExprVar(scip, &yexpr[1], vty, NULL, NULL));
                SCIP_CALL(SCIPcreateExprSum(scip, &expr1, 2, xexpr, coefs, 0.0, NULL, NULL));
                SCIP_CALL(SCIPcreateExprSum(scip, &expr2, 2, yexpr, coefs, 0.0, NULL, NULL));
                SCIP_CALL(SCIPcreateExprPow(scip, &powexpr1, expr1, 2.0, NULL, NULL));
                SCIP_CALL(SCIPcreateExprPow(scip, &powexpr2, expr2, 2.0, NULL, NULL));
                coefs[0] = 1.0;coefs[1] = 1.0;
                sumexpr[0] = powexpr1;
                sumexpr[1] = powexpr2;
                SCIP_CALL(SCIPcreateExprSum(scip, &lastpvrs, 2, sumexpr, coefs, SCIP_Real(-len2), NULL, NULL));
                SCIP_CALL(SCIPcreateExprPow(scip, &lastpvrs1, lastpvrs, 2, NULL, NULL));
                //add cons to scip
                SCIP_EXPR *finexpr[2];
                SCIP_CALL(SCIPcreateExprVar(scip, &finexpr[0], scip_var_m[i], NULL, NULL));
                finexpr[1] = lastpvrs1;
                coefs[0] = 1.0;coefs[1] = -1.0;
                SCIP_EXPR *lastypvrs2;
                SCIP_CALL(SCIPcreateExprSum(scip, &lastypvrs2, 2, finexpr, coefs, 0.0, NULL, NULL));
                SCIP_CALL(SCIPcreateConsBasicNonlinear(scip, &cons1, "consname1", lastypvrs2, 0.0, 0.0));
           //     SCIP_CALL(SCIPaddCons(scip, cons1));

                /* release expressions */
                SCIP_CALL( SCIPreleaseExpr(scip, &expr1) );
                SCIP_CALL( SCIPreleaseExpr(scip, &expr2) );
                SCIP_CALL( SCIPreleaseExpr(scip, &powexpr1) );
                SCIP_CALL( SCIPreleaseExpr(scip, &powexpr2) );
                SCIP_CALL( SCIPreleaseExpr(scip, &lastpvrs) );
                SCIP_CALL( SCIPreleaseExpr(scip, &lastpvrs1) );
                SCIP_CALL( SCIPreleaseCons(scip, &cons1) );
            }
        }

        // set SCIP parameters
        double tolerance = 1e-7;
        SCIP_CALL(SCIPsetRealParam(scip, "numerics/feastol", tolerance));
        SCIP_CALL(SCIPsetRealParam(scip, "numerics/dualfeastol", tolerance));
        SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", -1));  // enable presolve
        double MIP_gap = 1e-4;
        SCIP_CALL(SCIPsetRealParam(scip, "limits/gap", MIP_gap));
        bool status = false;

        // this tells scip to start the solution process
        if (SCIPsolve(scip) == SCIP_OKAY)
        {
            // get the best found solution from scip
            SCIP_SOL *sol = SCIPgetBestSol(scip);
            if (sol)
            {
                // If optimal or feasible solution is found.
                std::vector<vec2> reg_polygon(polygon.size());
                for (std::size_t i = 0; i < polygon.size(); ++i)
                {
                    reg_polygon[i] = vec2(SCIPgetSolVal(scip, sol, scip_var_x[i * 2]),
                                          SCIPgetSolVal(scip, sol, scip_var_x[i * 2 + 1]));
                }
                polygon = reg_polygon;
                status = true;
            }
        }

        // report the status: optimal, infeasible, etc.
        SCIP_STATUS scip_status = SCIPgetStatus(scip);
        switch (scip_status)
        {
            case SCIP_STATUS_OPTIMAL:
                // provides info only if fails.
                break;
            case SCIP_STATUS_GAPLIMIT:
                // To be consistent with the other solvers.
                // provides info only if fails.
                break;
            case SCIP_STATUS_INFEASIBLE:
                std::cerr << "model was infeasible" << std::endl;
                break;
            case SCIP_STATUS_UNBOUNDED:
                std::cerr << "model was unbounded" << std::endl;
                break;
            case SCIP_STATUS_INFORUNBD:
                std::cerr << "model was either infeasible or unbounded" << std::endl;
                break;
            default:
                if (scip_status == SCIP_STATUS_TIMELIMIT)
                    std::cerr << "time limit reached" << std::endl;
                break;
        }

        SCIP_CALL(SCIPresetParams(scip));

        // since the SCIPcreateVar captures all variables, we have to release them now
        for (std::size_t i = 0; i < scip_var_x.size(); ++i)
            SCIP_CALL(SCIPreleaseVar(scip, &scip_var_x[i]));
        scip_var_x.clear();

        // the same for the constraints

        // after releasing all vars and cons we can free the scip problem
        // remember this has always to be the last call to scip
        SCIP_CALL(SCIPfree(&scip));

    }
    catch (std::exception e)
    {
        std::cerr << "Error code = " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Exception during optimization" << std::endl;
    }
#endif
    return false;
}
