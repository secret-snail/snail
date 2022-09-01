# Installation
This project uses git submodules. Pull with `git submodule update --init --recursive`.

Note you need the version of raspian with a UI (not lite) for GTK support.
You can upgrade lite to full.
After running the first cmake command below, check GTK is ON in command output.

First install opencv (in home directory)
```
# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip

# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.2.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.2.zip
unzip opencv.zip
unzip opencv_contrib.zip

# Create build directory and switch into it
mkdir -p build && cd build

# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.5.2/modules ../opencv-4.5.2

# Enable swap if on raspberry pi
sudo mkdir -p /var/cache/swap/
sudo dd if=/dev/zero of=/var/cache/swap/swap0 bs=64M count=64
sudo mkswap /var/cache/swap/swap0
sudo swapon /var/cache/swap/swap0
sudo swapon -s

# Build (takes like 5 hours)
cmake --build .

# Install
sudo make install
```

Next modify emp library. Need to change `snail/extern/emp-tool/emp-tool/utils/block.h`, the two places where it says
```
#elif __aarch64__
```
Change to:
```
#elif __arm__
```
Also need to modify `snail/extern/emp-tool/emp-tool/utils/aes.h` but in this case you want the x86 code on line 106 to run and not the __aarch64__.
This is because ssetoneon takes care of transforming compiler intrinsics for you as the aarch64 ones don't work (no crypto built into pi g++ compiler.)
Be sure to also comment out the __attribute__ line

Install emp-tool deps
```
sudo apt install libssl-dev
```

Then build the source. Note the build directory is different than OpenCV's.
```
cd snail/
cmake -S . -B ./build
cmake --build ./build
```

Binary is located in `snail/build/bin/snail`

# Setup
First check (and maybe increase) camera resolution.

```
sudo v4l2-ctl -V
sudo v4l2-ctl -v width=1024,height=768
```

Save an image of a "ChArUco" board (chessboard of ArUco markers)
```
snail -c1
```

Print a photo of the board (as large as possible), then calibrate the pi cam. Should only take a few frames.
```
snail -c2
```

# Run
Run pose estimation. Press ESC to stop.
```
snail -c3
```

# Notes
See more aruco examples in `opencv_contrib-4.5.2/modules/aruco/samples/`
