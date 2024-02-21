FROM registry.gitlab.com/inivation/infra/docker-files/ubuntu:22.04

# Install prerequisites
RUN apt update
RUN apt-get -y install libcaer-dev dv-runtime-dev

# Copy dv-imu-cam-calibration inside docker
ADD dv-imu-cam-calibration-internal/ /dv-imu-cam-calibration-internal

# Build samples
RUN mkdir dv-imu-cam-calibration-internal/build
WORKDIR /dv-imu-cam-calibration-internal/build
RUN cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-11 -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 -DCMAKE_INSTALL_PREFIX=/usr -DIMU_CAM_CALIB_ENABLE_SAMPLES=ON ..
RUN make -j7
