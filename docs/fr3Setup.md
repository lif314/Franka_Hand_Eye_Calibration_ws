# FR3 Setup

## Network

## Install [libfranka](https://github.com/frankaemika/libfranka)

- Removing existing installations 
```shell
sudo apt remove "*libfranka*"
```

- Install dependencies
```shell
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```

- Clone `libfranka`
```shell
git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
```

- Build
```shell
cd libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```

- Build a Debian package
```shell
cpack -G DEB
```

- Install `libfranka`
```shell
sudo dpkg -i libfranka*.deb
```

## Install [franka_ros](https://github.com/frankaemika/franka_ros)
- Enter the `src` directory
```shell
cd path/to/Franka_Hand_Eye_Calibration_ws/src
```

- Clone
```shell
git clone --recursive https://github.com/frankaemika/franka_ros.git --branch noetic-devel
# OR
git clone --recursive https://github.com/frankaemika/franka_ros --branch noetic-devel
```

- Install any missing dependencies and build the packages
```shell
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
```

- Build
```shell
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```

## Setting up the real-time kernel
- Install the necessary dependencies
```shell
sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
```

- ref:
    - FCI docs: https://frankaemika.github.io/docs/installation_linux.html