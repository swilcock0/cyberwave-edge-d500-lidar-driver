# Stage 1: Build the C++ SDK and our wrapper
FROM ros:humble-ros-base AS builder

WORKDIR /build

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Clone the Lidar SDK from GitHub
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk.git /build/ldlidar_stl_sdk

# Copy our wrapper
COPY cyberwave_edge_d500_lidar_driver/lidar_wrapper.cpp /build/lidar_wrapper.cpp

# Build the wrapper as a shared library
# Fix: Include linux/types.h to resolve missing __u32/__u64 types
# Fix: Include pthread.h in log_module.h for mutex functions
WORKDIR /build/ldlidar_stl_sdk
RUN sed -i '1i #include <linux/types.h>' ldlidar_driver/include/serialcom/serial_interface_linux.h && \
    sed -i '1i #include <pthread.h>' ldlidar_driver/include/logger/log_module.h && \
    sed -i 's/cmake /cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON /g' auto_build.sh && \
    printf "0\n0\n" | bash ./auto_build.sh

# Compile our shared library wrapper, linking with the SDK statically so it is self-contained
WORKDIR /build
RUN g++ -O3 -shared -fPIC \
    -I/build/ldlidar_stl_sdk/ldlidar_driver/include/core \
    -I/build/ldlidar_stl_sdk/ldlidar_driver/include/dataprocess \
    -I/build/ldlidar_stl_sdk/ldlidar_driver/include/filter \
    -I/build/ldlidar_stl_sdk/ldlidar_driver/include/logger \
    -I/build/ldlidar_stl_sdk/ldlidar_driver/include/networkcom \
    -I/build/ldlidar_stl_sdk/ldlidar_driver/include/serialcom \
    lidar_wrapper.cpp \
    -o liblidar_wrapper.so \
    /build/ldlidar_stl_sdk/build/libldlidar_driver.a \
    -lpthread

# Stage 2: Final runtime image
# Using ROS2 Humble base image for native rclpy support
FROM ros:humble-ros-base AS base

WORKDIR /app

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Copy the built shared library
COPY --from=builder /build/liblidar_wrapper.so /usr/local/lib/
# Ensure the dynamic linker can find it
ENV LD_LIBRARY_PATH=/usr/local/lib

# Install Cyberwave SDK and other Python requirements
# We use --break-system-packages (or install in a venv) because ROS images
# manage their own system python
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY . .

ENV PYTHONUNBUFFERED=1

# Ensure ROS2 environment is sourced before running the driver
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 -m cyberwave_edge_d500_lidar_driver.main"]
