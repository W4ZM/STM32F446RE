# STM32F446RE-PROJECT
in this branch, i implemented the simples form of retargeting standard output, to send debugging informations to the terminal through UART protocol.

## Requirements  
- Linux OS
- [openocd](https://github.com/openocd-org/openocd)
- Cmake
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## How to build  
```bash
git clone https://github.com/W4ZM/STM32F446RE-PROJECTS.git
cd STM32F446RE-PROJECTS
git checkout UART-debugging
mkdir build && cd build
cmake ..
make
```
## Flash the firmware and connect
```bash
make flash && cu -l /dev/ttyACM0 -s 38400
```
