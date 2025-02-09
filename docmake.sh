#!/bin/bash

# Install necessary deps
sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib


if [ -d "build" ]; then
  rm -rf "build"
  echo "Deleted existing build directory"
fi

mkdir "build"
echo "Created build directory"

cd build 
cmake -DPICO_BOARD=pico2 ..