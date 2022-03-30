# Installation
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

# Build (takes like 5 hours)
cmake --build .

# Install
sudo make install
```

Then build the source
```
cd snail/
mkdir build
cd build
cmake ..
make
```

Binary is located in `snail/build/bin/snail`

# Setup
Save an image of a "ChArUco" board (chessboard of ArUco markers)
```
snail -c1
```

Print a photo of the board, then calibrate the pi cam. Should only take a few frames.
```
snail -c2
```

# Run
Run pose estimation
```
snail -c3
```

# Notes
See more aruco examples in `opencv_contrib-4.5.2/modules/aruco/samples/`
