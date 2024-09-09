FROM ubuntu:24.04 as build-base
ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get install -y --no-install-recommends \
    build-essential \
    automake \
    autoconf \
    ca-certificates \
    cmake \
    pkg-config \
    libgomp1 libomp-dev \
    # 
    clang-format clang-tidy clang-tools clang clangd libc++-dev libc++1 \
    libc++abi-dev libc++abi1 libclang-dev libclang1 liblldb-dev libllvm-ocaml-dev \
    libomp-dev libomp5 lld lldb llvm-dev llvm-runtime llvm python3-clang \
    #
    libva-dev libvdpau-dev xkb-data \
    libx11-xcb-dev libfontenc-dev libice-dev libsm-dev libxaw7-dev \
    libxcomposite-dev libxcursor-dev libxdamage-dev libxext-dev libxfixes-dev \
    libxi-dev libxinerama-dev libxkbfile-dev libxmu-dev libxmuu-dev libxpm-dev \
    libxrandr-dev libxrender-dev libxres-dev libxss-dev libxt-dev libxtst-dev \
    libxv-dev libxxf86vm-dev libxcb-glx0-dev libxcb-render0-dev libxcb-render-util0-dev \
    libxcb-xkb-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev \
    libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-xfixes0-dev \
    libxcb-xinerama0-dev libxcb-dri3-dev uuid-dev libxcb-cursor-dev libxcb-dri2-0-dev \
    libxcb-dri3-dev libxcb-present-dev libxcb-composite0-dev libxcb-ewmh-dev libxcb-res0-dev \
    libxcb-util-dev libxcb-util0-dev \
    python3-venv python3-pip && \
    # clean up
    rm -rf /var/lib/apt/lists/*

FROM build-base as conan-step

# change to bash (needed for conan to build correctly)
SHELL ["/bin/bash", "-c"]

ENV PATH="/venv/bin:$PATH"

WORKDIR /city3d

COPY ./src/ ./src
COPY ./CMakeLists.txt .
COPY ./conanfile.py .

RUN python3 -m venv /venv && \
    /venv/bin/pip install conan && \
    conan profile detect && \
    #  -c tools.system.package_manager:mode=install 
    conan install . --build=missing -verror

FROM conan-step as build-step
# change to bash (needed for conan to build correctly)
SHELL ["/bin/bash", "-c"]

WORKDIR /city3d/build/Release

RUN ldconfig && \
    source ./generators/conanbuild.sh && \
    cmake ../.. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release && \
    cmake --build .

FROM ubuntu:24.04 as city3d

WORKDIR /city3d

# copy compiled assets from build-step
COPY --from=build-step /city3d/build/Release/bin /city3d

CMD [ "./city3d -h" ]
