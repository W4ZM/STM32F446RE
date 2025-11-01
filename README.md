# STM32F446RE-PROJECTS
In this branch, i implemented the simplest form of memory-to-peripheral (USART2) DMA, to transfer a message to the terminal through UART protocol.

## Requirements  
- Linux OS
- [openocd](https://github.com/openocd-org/openocd)
- Cmake
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## How to build  
```bash
git clone https://github.com/W4ZM/STM32F446RE-PROJECTS.git
cd STM32F446RE-PROJECTS
git checkout UART-DMA
mkdir build && cd build
cmake ..
make
```
## Flash the firmware and connect
```bash
make flash && cu -l /dev/ttyACM0 -s 38400
```
