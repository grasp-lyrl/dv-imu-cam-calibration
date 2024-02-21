# Usage inside Docker

Calibration samples can also be used via Docker from [this Dockerfile](/docker/Dockerfile). (It uses
[this other Dockerfile from our gitlab](https://gitlab.com/inivation/infra/docker-files/-/blob/master/ubuntu_lts_2204/Dockerfile?ref_type=heads)
as a base)

Build it and run it with the following command line instructions:

```shell
# Go to parent folder from dv-imu-cam-calibration-internal (this is necessary to copy repo inside docker)
cd ..
# Build docker image
docker build -t run_calibration -f dv-imu-cam-calibration-internal/docker/Dockerfile .
# Run docker image with the desired command. Note that the 'data' folder is mounted for read and write operations
docker run -v /path/to/data/folder/:/data run_calibration <your_command>
```

Currently:

- the [Dockerfile](/docker/Dockerfile) sets the working directory inside the `build` directory of its copy of
  `dv-imu-cam-calibration-internal` repo.
- the data folder is mounted under `/data`

Therefore, to run a calibration sample, `<your_command>` should be replaced by something similar to the following:

- To run calibrations for all device folders:

```shell
./samples/camera-calibration/run-camera-calibration-all-devices -f /data/recordings/ -o /data/intrinsic-results/
```

- To run calibrations for all patterns of one device folder:

```shell
./samples/camera-calibration/run-camera-calibration-all-patterns -f /data/recordings/dvxplorer/ -o /data/intrinsic-results/dvxplorer/ --width 640 --height 480
```

- To run calibrations for all recordings of one device-pattern combination folder:

```shell
./samples/camera-calibration/run-camera-calibration-all-recordings -f /data/recordings/dvxplorer/april-tag-0.5m/ -o /data/intrinsic-results/dvxplorer/april-tag-0.5m/ -p /data/recordings/dvxplorer/april-tag-0.5m/april-tag-0.5m.json --width 640 --height 480
```

- To run calibrations for all reconstruction methods of one device-pattern-recording combination folder:

```shell
./samples/camera-calibration/run-camera-calibration-all-recordings -f /data/recordings/dvxplorer/april-tag-0.5m/rec1/reconstructed/ -o /data/intrinsic-results/dvxplorer/april-tag-0.5m/rec1 -p /data/recordings/dvxplorer/april-tag-0.5m/april-tag-0.5m.json --width 640 --height 480
```

- To run calibration for a single reconstruction method of one device-pattern-recording combination folder:

```shell
./samples/camera-calibration/run-camera-calibration -f /data/recordings/dvxplorer/april-tag-0.5m/rec1/reconstructed/E2VID+_april_0.5m-2023_11_24_12_14_02.aedat4 -o /data/intrinsic-results/dvxplorer/april-tag-0.5m/rec1/E2VID+_april_0.5m-2023_11_24_12_14_02.json -p /data/recordings/dvxplorer/april-tag-0.5m/april-tag-0.5m.json --width 640 --height 480
```
