# =============================================================================
# LiteVO — Development Docker image
# =============================================================================
# Build:
#   docker build -t litevo -f docker/Dockerfile .
#
# Run:
#   docker run --rm -v $(pwd)/data:/workspace/data litevo litevo_cli

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
    # OpenCV
    libopencv-dev \
    # Eigen3
    libeigen3-dev \
    # spdlog
    libspdlog-dev \
    # yaml-cpp
    libyaml-cpp-dev \
    # BLAS/LAPACK (for Ceres & g2o)
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev \
    # Google Log (for Ceres)
    libgoogle-glog-dev \
    # Python for tools
    python3 \
    python3-pip \
    python3-venv \
    # Clean up
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ── Sophus (from source) ─────────────────────────────────────────────────────
RUN git clone --depth 1 --branch v1.22.10 \
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
RUN git clone --depth 1 --branch v0.1 \
        https://github.com/fbachraty/FBOW.git /tmp/FBOW \
    && cd /tmp/FBOW \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/FBOW

# ── Update library cache ──────────────────────────────────────────────────────
RUN ldconfig

# ── Python tools dependencies ─────────────────────────────────────────────────
RUN pip3 install --no-cache-dir numpy matplotlib pandas

# ── Copy LiteVO source ────────────────────────────────────────────────────────
COPY . /opt/litevo

# ── Build LiteVO ──────────────────────────────────────────────────────────────
WORKDIR /opt/litevo/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DLITEVO_BUILD_TESTS=OFF \
    -DLITEVO_BUILD_APPS=ON \
    -DLITEVO_ENABLE_SOPHUS=ON \
    -DLITEVO_ENABLE_CERES=ON \
    -DLITEVO_ENABLE_G2O=ON \
    -DLITEVO_ENABLE_FBOW=ON \
    -DLITEVO_ENABLE_SPDLOG=ON \
    -DLITEVO_ENABLE_YAML=ON \
    && make -j$(nproc)

# ── Install ───────────────────────────────────────────────────────────────────
RUN make install

# ── Entry point ───────────────────────────────────────────────────────────────
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/litevo_cli"]
CMD ["--help"]
