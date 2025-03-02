#!/bin/bash

# Install necessary deps
sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

# Clone all the dependencies into the dep folder
if [ -d "dep" ]; then
  echo "Dependency folder exists"
else
  mkdir "dep"
  echo "Created dependency folder. Cloning repos..."
  cd dep
  git clone https://github.com/raspberrypi/pico-sdk.git
  cd pico-sdk
  git submodule update --init --recursive
  cd ..
  # git clone https://github.com/earlephilhower/arduino-pico.git
  # cd arduino-pico
  # git submodule update --init --recursive
  # cd tools
  # python3 get.py
  # cd ../..
  # git clone https://github.com/adafruit/Adafruit_SHT4X.git
  # git clone https://github.com/WiringPi/WiringPi.git
  curl -L https://github.com/WiringPi/WiringPi/archive/refs/tags/3.10.tar.gz -o WiringPi-3.10.tar.gz
  tar -xzf WiringPi-3.10.tar.gz

  # copy dep\pico-sdk\external\pico_sdk_import.cmake to current directory
  cp pico-sdk/external/pico_sdk_import.cmake ../
  cd ..
fi




if [ -d "build" ]; then
  rm -rf "build"
  echo "Deleted existing build directory"
fi

mkdir "build/"
echo "Created build directory"

cd build 
cmake -DPICO_BOARD=pico2 ..
make -j4