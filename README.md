# STM32F446RE-PROJECTS
A bunch of stm32f4 dev board projects without using the CubeIDE, the first project will be blinking LED, more projects will be added as branches to this one.

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
