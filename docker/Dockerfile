###############
#
# SPT-based 可視領域計算法のDocker環境構築
#
###############

##### ベースイメージ #####
FROM ubuntu:22.04

# パッケージ管理システム
RUN apt-get update && apt-get install -y \
    apt-file \
    iproute2 \
    wget \
    software-properties-common

# C++ライブラリ
RUN apt-get update && apt-get install -y \
    build-essential \
    gdb \
    libboost-all-dev \
    libgmp3-dev \
    libmpfr-dev \
    libeigen3-dev \
    libcgal-dev \
    libomp-dev \
    libtbb-dev
     
# cmake
RUN apt-get update && apt-get install -y \
    cmake \
    git

# pagmo のインストール
WORKDIR /tmp
RUN git clone https://github.com/esa/pagmo2.git && \
    mkdir pagmo2/build
WORKDIR /tmp/pagmo2/build
RUN cmake .. \
-DCMAKE_INSTALL_PREFIX=/usr \
    -DCMAKE_BUILD_TYPE=Release \
    -DPAGMO_BUILD_TESTS=OFF \
    -DPAGMO_BUILD_BENCHMARKS=OFF \
    && make -j$(nproc) \
    && make install
WORKDIR /
RUN rm -rf /tmp/pagmo2

# 不要になったキャッシュを削除してイメージサイズを減らす
RUN apt-get clean \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

##### コンテナ起動時に実行するコマンド #####
CMD ["/bin/bash"]