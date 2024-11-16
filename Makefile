# Makefile for BELUGA Project
#
# Author: Marwan
# Description: This Makefile automates the process of fetching, building, and linking 
#              the dependencies needed to build our code for BELUGA. Dependencies are 
#              managed within the 'dep/' directory.
#
# Usage:
# - `make` to download, build, and link the project.
# - `make clean` to remove object and binary files.
# - `make distclean` to remove all build artifacts, including the WiringPi library.


WIRINGPI_TAR = 3.10.tar.gz
WIRINGPI_URL = https://github.com/WiringPi/WiringPi/archive/refs/tags/3.10.tar.gz
WIRINGPI_DIR = dep/WiringPi-3.10
WIRINGPI_LIB = $(WIRINGPI_DIR)/wiringPi/libwiringPi.so.3.10

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
CXX = g++

CPP_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(CPP_FILES))
TARGET = $(BIN_DIR)/motor_control

CXXFLAGS = -std=c++20 -Wall -Wextra -O2 -I$(WIRINGPI_DIR)/wiringPi
LDFLAGS = -L$(WIRINGPI_DIR)/wiringPi -lwiringPi

all: $(WIRINGPI_LIB) $(TARGET)

$(WIRINGPI_TAR):
	cd dep && wget $(WIRINGPI_URL) -O $(WIRINGPI_TAR)

$(WIRINGPI_DIR): $(WIRINGPI_TAR)
	cd dep && tar -xzf $(WIRINGPI_TAR)

$(WIRINGPI_LIB): $(WIRINGPI_DIR)
	cd $(WIRINGPI_DIR) && ./build

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJ_FILES) | $(BIN_DIR)
	$(CXX) $(OBJ_FILES) -o $@ $(LDFLAGS)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

clean-wiringpi:
	cd $(WIRINGPI_DIR)/wiringPi && make clean

distclean: clean clean-wiringpi
	rm -rf $(WIRINGPI_DIR) $(WIRINGPI_TAR)