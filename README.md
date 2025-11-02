# STM32F446RE
Simple examples without using the CubeIDE, this branch is for blinking LD2, check other branches for more examples.

## Requirements  
- Linux OS
- [openocd](https://github.com/openocd-org/openocd)
- Cmake
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## How to build  
```bash
git clone https://github.com/W4ZM/STM32F446RE-PROJECTS.git
cd STM32F446RE-PROJECTS
mkdir build && cd build
cmake ..
make
```
## Flash the firmware  
```bash
make flash
```
