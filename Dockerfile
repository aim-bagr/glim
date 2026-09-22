FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH="/opt/glim/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"

# 1. Install prerequisites & configure Koide3 PPA
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gpg \
    ca-certificates \
    software-properties-common \
    && curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null \
    && echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | tee /etc/apt/sources.list.d/koide3_ppa.list \
    && apt-get update

# 2. Install all build & runtime dependencies (Zero ROS dependencies)
RUN apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libboost-all-dev \
    libspdlog-dev \
    libfmt-dev \
    libmetis-dev \
    libglfw3-dev \
    libglm-dev \
    libomp-dev \
    libpng-dev \
    libjpeg-dev \
    libopencv-dev \
    libzstd-dev \
    liblz4-dev \
    libeigen3-dev \
    libnanoflann-dev \
    libgtsam-notbb-dev \
    libgtsam-points-cuda12.2-dev \
    libiridescence-dev \
    zenity \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 2b. Install web service microservice dependencies
RUN pip3 install --no-cache-dir fastapi "uvicorn[standard]" websockets pydantic

# 3. Copy source tree
WORKDIR /opt/glim
COPY . /opt/glim

# 4. Build GLIM and standalone MCAP runner
WORKDIR /opt/glim/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_CUDA=ON \
    -DBUILD_WITH_VIEWER=ON \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DBUILD_STANDALONE_MCAP=ON \
    && cmake --build . -j$(nproc) \
    && ln -s /opt/glim/build/glim_mcap /usr/local/bin/glim_mcap \
    && ln -s /opt/glim/build/glim_offline_viewer /usr/local/bin/glim_offline_viewer \
    && chmod -R a+rX /opt/glim

EXPOSE 8080

WORKDIR /opt/glim
CMD ["glim_mcap", "--help"]
