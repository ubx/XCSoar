# XCSoar Docker Image

This Docker Image when built, will compile XCSoar for several targets in a clean room environment.


## Currently Supported Targets

- UNIX (linux & co)
- ANDROID
- PC
- KOBO
- DOCS

## Instructions

The container itself is readonly. The build results will appear in `./output/`.

To build the container:
```
docker build \
    --file ide/docker/Dockerfile \
    -t xcsoar/xcsoar-build:latest ./ide/
```

To run the container interactivly:
```
docker run \
    --mount type=bind,source="$(pwd)",target=/opt/xcsoar \
    -it xcsoar/xcsoar-build:latest /bin/bash
```

To run the ANDROID build:
```
docker run \
    --mount type=bind,source="$(pwd)",target=/opt/xcsoar \
    -it xcsoar/xcsoar-build:latest xcsoar-compile ANDROID
```

To run the PI build:
```
docker run \
    --mount type=bind,source="$(pwd)",target=/opt/xcsoar \
    --mount type=bind,source=/media/andreas/rootfs,target=/opt/pi/root \
    -it xcsoar/xcsoar-build:latest xcsoar-compile PI
```
