cmake_minimum_required(VERSION 3.28.3)

# Tell CMake to use our toolchain file
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain.cmake")

project(blink)

set(CMAKE_C_STANDARD "23") # C23

file(GLOB_RECURSE SRC
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
)

add_executable("${PROJECT_NAME}" ${SRC})

target_include_directories("${PROJECT_NAME}" PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include"
    "${CMAKE_CURRENT_LIST_DIR}/include"
)

# Produce a file 'blink.elf' that can be uploaded to the MCU
set_target_properties("${PROJECT_NAME}" PROPERTIES OUTPUT_NAME "blink.elf")

target_compile_definitions(${PROJECT_NAME} PRIVATE STM32F446xx)

# Target compile options
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -Wall
    -g0
)
    
# Target link options
target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker_script.ld
    -Wl,-Map=blink.map
    -Wl,--gc-sections
    -static
    --specs=nano.specs
    -nostartfiles
    -Wl,--strip-all
)

# simple "flash" phony target
add_custom_target(flash
  COMMAND openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program blink.elf verify reset exit"
  DEPENDS "${CMAKE_PROJECT_NAME}"
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)