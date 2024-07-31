### City3D: Large-scale Building Reconstruction from Airborne LiDAR Point Clouds

![scene image](./images/scene.png)

City3D implements the hypothesis-and-selection based building reconstruction method described in the following [paper](https://www.mdpi.com/2072-4292/14/9/2254):

```
Jin Huang, Jantien Stoter, Ravi Peters, Liangliang Nan.
City3D: Large-scale Building Reconstruction from Airborne LiDAR Point Clouds.
Remote Sensing. 14(9), 2254, 2022.
```

This implementation is based on [PolyFit](https://github.com/LiangliangNan/PolyFit).

Despite being a research prototype, City3D has been used to create city-scale building models. [This article](https://research.geodan.nl/the-possibilities-and-limitations-of-city3d-as-large-scale-3d-building-reconstruction-model/) explores its potential and limitations from a user's perspective, comparing results in Dutch and French datasets.

---

### Obtaining City3D

You can build City3D from the source code˙

- Download the [source code](https://github.com/tudelft3d/City3D).
- Dependencies (Attention for Windows users: ALL dependencies must be for `x64`)

  - [CGAL](http://www.cgal.org/index.html) (v5.6, v5.5 and v5.4 have been tested). Newer versions of CGAL (v5.6 and later) are always preferred for better performance.
  - [OpenCV](https://opencv.org/releases/) (v4.0 and later, only the main modules are needed).

- Optional dependencies (see [About the solvers](#about-the-solvers)):

  - [Gurobi](https://www.gurobi.com/). **Note for Linux users:** You may have to build the Gurobi library (`libgurobi_c++.a`)
    because the prebuilt one in the original package might NOT be compatible with your compiler. To do so, go to `PATH-TO-GUROBI/src/build`
    and run `make`. Then replace the original `libgurobi_c++.a` (in the `lib` directory) with your generated file.

- Build

  There are many options to build City3D. Choose one of the following (not an exhaustive list):

  - Option 1 (purely on the command line): Use CMake to generate Makefiles and then `make` (on Linux/macOS) or `nmake`(on Windows with Microsoft
    Visual Studio).

    - On Linux or macOS
      ```
      $ cd path-to-root-dir-of-City3D
      $ mkdir Release
      $ cd Release
      $ cmake -DCMAKE_BUILD_TYPE=Release ..
      $ make
      ```
    - On Windows with Microsoft Visual Studio, use the `x64 Native Tools Command Prompt for VS XXXX` (don't use the x86 one), then
      ```
      $ cd path-to-root-dir-of-City3D
      $ mkdir Release
      $ cd Release
      $ cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
      $ nmake
      ```

  - Option 2: Use any IDE that can directly handle CMakeLists files to open the `CMakeLists.txt` in the **root** directory of
    City3D. Then you should have obtained a usable project and just build it. I recommend using
    [CLion](https://www.jetbrains.com/clion/) or [QtCreator](https://www.qt.io/product).
    For Windows users: your IDE must be set for `x64`.

  - Option 3: Use CMake-Gui to generate project files for your favorite IDE. Then load the project to your IDE and build it.
    For Windows users: your IDE must be set for `x64`.

  Don't have any experience with C/C++ programming? Then check this <a href="https://github.com/LiangliangNan/Easy3D/blob/main/HowToBuild.md">How to build</a> I wrote for [Easy3D](https://github.com/LiangliangNan/Easy3D).

### Run City3D

```shell
DOCKER_BUILDKIT=1 docker build --pull --rm -f "Dockerfile" -t city3d:latest .

docker images city3d

docker run --rm -it \
    -v ${PWD}/data:/data \
    city3d:latest \
    ./city3d_cli -f /data/001_footprint.obj -p /data/001.ply -o /data/001_result.obj
```

---

### Data

The method has been evaluated on approximately 20,000 buildings, resulting in a new dataset consisting of the original point clouds
and the reconstructed 3D models of all these buildings. The complete dataset can be found [here](https://github.com/yidahuang/City3D_dataset).

This repository has included a few buildings from the above dataset for testing, which can be found in the [data](https://github.com/tudelft3d/City3D/tree/main/data) directory.

In our implementation, we assume the footprint of a building is a simple polygon. For a set of buildings, their footprint data consists of
a set of polygonal faces, and these faces are stored in an [OBJ file](https://en.wikipedia.org/wiki/Wavefront_.obj_file) with Z coordinates set to 0.

Sources of footprint data may include:

- The cadastre of the city/country. Many countries maintain footprint data through their cadastre, which may be publicly available or can be obtained
  by contacting the relevant authorities.
- Open data platforms such as [OpenStreetMap](https://tudelft3d.github.io/3dfier/building_footprints_from_openstreetmap.html) may offer footprint data.

In case you have a point cloud of a scene containing multiple buildings, but don't have access to the footprint of the buildings, you will need to
segment out the individual buildings either manually or using an automatic approach. There are various published papers addressing instance segmentation
of urban buildings from point clouds. You can explore these methods to find one that fits your purpose, and then [CLI_Example_2](https://github.com/tudelft3d/City3D/tree/main/code/CLI_Example_2)
will be suitable for the reconstruction.

---

### About the solvers

This demo program can use either the SCIP solver or the commercial solver [Gurobi](https://www.gurobi.com/) for the [core optimization](https://github.com/tudelft3d/City3D/blob/main/code/method/face_selection_optimization.cpp) step.
The entire source code of the SCIP solver is already included in this repository.

The Gurobi solver is faster than SCIP and is thus highly recommended. To use Gurobi, install it first and make sure
the headers and libraries of Gurobi can be found by CMake. This can be done by specifying the paths of Gurobi in [FindGUROBI.cmake](./code/cmake/FindGUROBI.cmake).
Note: you need to [obtain a license](https://www.gurobi.com/downloads/end-user-license-agreement-academic/) to use Gurobi, which is free for academic use.

---

### Citation

If you use the code/program (or part) of City3D in scientific work, please cite our paper:

```bibtex
@Article{HuangCity3d_2022,
    AUTHOR = {Huang, Jin and Stoter, Jantien and Peters, Ravi and Nan, Liangliang},
    TITLE = {City3D: Large-Scale Building Reconstruction from Airborne LiDAR Point Clouds},
    JOURNAL = {Remote Sensing},
    VOLUME = {14},
    YEAR = {2022},
    NUMBER = {9},
    ARTICLE-NUMBER = {2254},
}
```

---

### License

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 3 of the License or (at your option) any later version. The full text of the license can be found in the accompanying LICENSE file.

---

Should you have any questions, comments, or suggestions, please feel free to contact me at:
J.Huang-1@tudelft.nl

**_Jin Huang_**

https://yidahuang.github.io/

June 28, 2022

Copyright (C) 2022
