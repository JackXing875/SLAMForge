# =============================================================================
# SLAMForge — Development Docker image
# =============================================================================
# Build:
#   docker build -t slamforge -f docker/Dockerfile .
#
# Run:
#   docker run --rm -v $(pwd)/data:/images -v $(pwd)/output:/output \
#       slamforge run --config /opt/slamforge/config/kitti.yaml --input /images \
#       --output /output/trajectory.txt

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    ccache \
    libopencv-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    python3 \
    python3-pip \
    python3-venv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ── Sophus (from source) ─────────────────────────────────────────────────────
RUN git clone --depth 1 --branch 1.22.10 \
        https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/Sophus

# ── Ceres Solver (from source) ────────────────────────────────────────────────
RUN git clone --depth 1 --branch 2.2.0 \
        https://ceres-solver.googlesource.com/ceres-solver /tmp/ceres-solver \
    && cd /tmp/ceres-solver \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_TESTING=OFF \
                -DBUILD_EXAMPLES=OFF \
                -DBUILD_BENCHMARKS=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/ceres-solver

# ── g2o (from source) ────────────────────────────────────────────────────────
RUN git clone --depth 1 --branch 20230223_git \
        https://github.com/RainerKuemmerle/g2o.git /tmp/g2o \
    && cd /tmp/g2o \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_WITH_MARCH_NATIVE=OFF \
                -DG2O_BUILD_EXAMPLES=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/g2o

# ── FBOW (Fast Bag-of-Words, from source) ─────────────────────────────────────
RUN git clone --depth 1 --branch master \
        https://github.com/rmsalinas/fbow.git /tmp/fbow \
    && cd /tmp/fbow \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/fbow

# ── Update library cache ──────────────────────────────────────────────────────
RUN ldconfig

# ── Python tools dependencies ─────────────────────────────────────────────────
RUN pip3 install --no-cache-dir numpy matplotlib pandas

# ── Copy SLAMForge source ────────────────────────────────────────────────────────
COPY . /opt/slamforge

# ── Build SLAMForge ──────────────────────────────────────────────────────────────
WORKDIR /opt/slamforge/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DSLAMFORGE_BUILD_TESTS=OFF \
    -DSLAMFORGE_BUILD_APPS=ON \
    -DSLAMFORGE_ENABLE_SOPHUS=ON \
    -DSLAMFORGE_ENABLE_CERES=ON \
    -DSLAMFORGE_ENABLE_G2O=ON \
    -DSLAMFORGE_ENABLE_FBOW=ON \
    -DSLAMFORGE_ENABLE_SPDLOG=ON \
    -DSLAMFORGE_ENABLE_YAML=ON \
    && make -j$(nproc)

# ── Install ───────────────────────────────────────────────────────────────────
RUN cmake --install .

# ── Entry point ───────────────────────────────────────────────────────────────
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/slamforge_cli"]
CMD ["--help"]
