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
    git \
    # clean up
    && rm -rf /var/lib/apt/lists/*

FROM build-base as build-step

RUN git clone -b 'main' --single-branch --depth 1 https://github.com/tudelft3d/City3D.git /src

WORKDIR /src

RUN mkdir ./build; \
    cd ./build; \
    echo "hello"; \
    cmake -DCMAKE_BUILD_TYPE=build ..; \
    make;

FROM ubuntu:24.04 as main

RUN apt-get update \
    && apt-get -y upgrade \
    # install dependencies needed to run City3D
    && apt-get install -y --no-install-recommends \
    libopengl0 \
    libglu1-mesa \
    libopencv-dev \
    # clean up
    && rm -rf /var/lib/apt/lists/*

# copy compiled assets from build-step
COPY --from=build-step /src/build/ /city3d

WORKDIR /city3d

CMD [ "./bin/CLI_Example_1" ]
