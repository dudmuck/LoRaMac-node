
# simple LoRa example
These added examples are **not** LoRaWAN, instead they use LoRa transceiver chip directly.

added applications:

    -DAPPLICATION="simple-tx"
    -DAPPLICATION="simple-rx"
    -DAPPLICATION="led-blink" 

added boards:

    -DBOARD="NUCLEO-L031K6"
    -DBOARD="NUCLEO-G071RB" 
    -DBOARD="NUCLEO-G070RB" 
    
already existing radio chips:

    -DMBED_RADIO_SHIELD="LR1110MB1XXS"
    -DMBED_RADIO_SHIELD="SX1262MBXCAS"
    -DMBED_RADIO_SHIELD="SX1261MBXBAS"
    
if you're using STM32L0xx MCU:

    in directory src/boards/mcu/stm32:
    git submodule update --init STM32CubeL0

**or**, if you're using STM32L1xx MCU:

    in directory src/boards/mcu/stm32:
    git submodule update --init STM32CubeL1

**or**, if you're using STM32L4xx MCU:

    in directory src/boards/mcu/stm32:
    git submodule update --init STM32CubeL4

**or**, if you're using STM32G0xx MCU:

    in directory src/boards/mcu/stm32:
    git submodule update --init STM32CubeG0

if you're using LR1110 radio chip:

    in directory src/radio/lr1110:
    git submodule update --init lr1110_driver
        
## running cmake
### running cmake on windows
create a ``.ps1`` script for running cmake, example to be run from your ``build`` directory:

    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_TOOLCHAIN_FILE="../cmake/toolchain-arm-none-eabi.cmake" `
        -DTOOLCHAIN_PREFIX="C:/Program Files (x86)/GNU Arm Embedded Toolchain/9 2020-q2-update" `
        -DAPPLICATION="simple-tx" `
        -DMBED_RADIO_SHIELD="SX1262MBXCAS" `
        -DUSE_RADIO_DEBUG="ON" `
        -DBOARD="NUCLEO-G071RB" `
     ..
     
 Above example uses ninja generator, if you dont have it, get ninja from https://ninja-build.org/
 
 Recommened to use [windows terminal](https://docs.microsoft.com/en-us/windows/terminal/) instead of powershell.
 
 If you've never run ``.ps1`` script before, you need to [allow them to run](https://superuser.com/questions/106360/how-to-enable-execution-of-powershell-scripts).
 
### running cmake on linux
create a ``.sh`` script for running cmake, example to be run from your ``build`` directory:

    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="../cmake/toolchain-arm-none-eabi.cmake" \
        -DTOOLCHAIN_PREFIX="/home/youruser/gcc-arm-none-eabi-9-2020-q2-update" \
        -DAPPLICATION="simple-tx" \
        -DMBED_RADIO_SHIELD="SX1262MBXCAS" \
        -DUSE_RADIO_DEBUG="ON" \
        -DBOARD="NUCLEO-G071RB" \
     ..

## building with IDE
install [cmake_ide_generator](https://github.com/dudmuck/cmake_ide_generator).

in your ``build`` directory, run ``query-cmake`` prior to running cmake, then after cmake completes, json will then be present, permitting IDE project files to be generated.
### building with stm32CubeIDE


in your ``build`` directory run `` stm32cubeide-parse-reply`` after running cmake

you will then see ``.project`` and ``.cproject`` files, which can be imported into IDE as existing project.

