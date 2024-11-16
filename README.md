# BELUGA-FW
All BELUGA related firmware code


# **Firmware Build System**

This build system automates the process of fetching, building, and linking dependencies using a `Makefile`.

---

## **Features**
- Automatically downloads, extracts, and builds dependencies (WiringPi, etc.)
- Compiles all C++ files in the `src/` directory.
- Links the compiled code with dep binaries to produce an executable.
- Includes clean targets to remove build artifacts.

---

## **Project Structure**
```
project/
├── src/                # C++ source files
├── obj/                # Compiled object files (created during build)
├── bin/                # Final executable (created during build)
├── dep/                # Dependency sources (fetched during build)
├── Makefile            # Build script
└── README.md           # Documentation
```

---

## **Build Instructions**
1. **Build the Project**:
   ```bash
   make
   ```
   - Downloads and builds WiringPi.
   - Compiles and links the motor control executable.

2. **Clean Build Artifacts**:
   ```bash
   make clean
   ```
   - Removes compiled files only.

3. **Full Cleanup**:
   ```bash
   make distclean
   ```
   - Removes all files, including WiringPi.

---

## **Requirements**
Ensure the following tools are installed:
- `make`
- `g++`
- `wget`
- `tar`