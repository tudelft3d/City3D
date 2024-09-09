# City3D Containerized

This repository is a fork of [City3D](https://github.com/tudelft3d/City3D) with the purpose of reducing the cumbersome C++ build process by using a Dockfile.
Additionally all code needed for the graphical user interface is removed because the intention is to run the resulting container image in a cloud/kubernetes environment.

## Get the Image

```shell
docker pull ghcr.io/baal-lgln/city3d:latest
```

## Local development and testing

Build the image with:

```shell
docker build --pull --rm -t city3d:latest .
```

Mount a volume with your data at `$(PWD)/data` and run the image with:

```shell
docker run --rm -it \
    -v ${PWD}/data:/data \
    city3d:latest \
    ./city3d_cli -f /data/001_footprint.obj -p /data/001.ply -o /data/001_result.obj
```

Some test data can be found in the [original repository](https://github.com/tudelft3d/City3D/tree/main/data).

## Local build

If you want to build the project yourself:

```shell
python3 -m venv .venv
source .venv/bin/activate # Linux
# source .venv/Scripts/activate # Windows
python3 -m pip install conan
conan install . --build=missing
cd build/Release
source ./generators/conanbuild.sh
cmake ../.. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build .
# run city3d
./bin/city3d -h
```