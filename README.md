# STM32F446RE-PROJECTS
In this branch, i used advanced-control timer (TIM1) along with DMA2 controller to toggle on/off LD2 without the intervention of the CPU. 

## Requirements  
- Linux OS
- [openocd](https://github.com/openocd-org/openocd)
- Cmake
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## How to build  
```bash
git clone https://github.com/W4ZM/STM32F446RE-PROJECTS.git
cd STM32F446RE-PROJECTS
git checkout TIMER-DMA
mkdir build && cd build
cmake ..
make
```
## Flash the firmware
```bash
make flash
```
