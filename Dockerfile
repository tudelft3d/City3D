FROM ubuntu:24.04 as build-base

RUN apt-get update \
    && apt-get -y upgrade \
    # install City3D dependencies
    && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    libboost-all-dev \
    libcgal-dev \
    libopencv-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libatlas3-base \
    libopengl0 \
    git \
    # clean up
    && rm -rf /var/lib/apt/lists/*

FROM build-base as build-step

# RUN git clone -b 'main' --single-branch --depth 1 https://github.com/tudelft3d/City3D.git /src
COPY ./code /src/code
COPY ./CMakeLists.txt /src/CMakeLists.txt

WORKDIR /src

RUN mkdir ./build; \
    cd ./build; \
    cmake -DCMAKE_BUILD_TYPE=Release ..; \
    make;

FROM ubuntu:24.04 as main

# copy compiled assets from build-step
COPY --from=build-step /src/build/bin /city3d
# copy necessary libs from build-base
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.6.0 /usr/local/lib/libopencv_imgproc.so.4.6.0
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0 /usr/local/lib/libopencv_core.so.4.6.0
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libGLX.so.0 /usr/local/lib/libGLX.so.0
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libatlas.so.3 /usr/local/lib/libatlas.so.3
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libtbb.so.12 /usr/local/lib/libtbb.so.12
COPY --from=build-base /usr/lib/x86_64-linux-gnu/liblapack.so.3 /usr/local/lib/liblapack.so.3
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libblas.so.3 /usr/local/lib/libblas.so.3
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libX11.so.6 /usr/local/lib/libX11.so.6
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libGLU.so.1 /usr/local/lib/libGLU.so.1
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libOpenGL.so.0 /usr/local/lib/libOpenGL.so.0
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libGLdispatch.so.0 /usr/local/lib/libGLdispatch.so.0
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libgfortran.so.5 /usr/local/lib/libgfortran.so.5
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libxcb.so.1 /usr/local/lib/libxcb.so.1
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libXau.so.6 /usr/local/lib/libXau.so.6
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libXdmcp.so.6 /usr/local/lib/libXdmcp.so.6
COPY --from=build-base /usr/lib/x86_64-linux-gnu/libbsd.so.0 /usr/local/lib/libbsd.so.0

RUN apt-get update \
    && apt-get -y upgrade \
    # clean up
    && rm -rf /var/lib/apt/lists/* \
    # create the necessary links and cache to recently copied shared libraries
    && ldconfig

WORKDIR /city3d

CMD [ "./CLI_Example_1" ]
