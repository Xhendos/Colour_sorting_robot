# Description

This directory contains all the documentation and information that is available about the colour sorting robot or any part that the colour sorting robot is using.

## Setup development environment on Linux

This section will describe the steps that you will need to perform to develop on the STM32F103C8T6. This will work on Linux machines only.

1. Get the [st-link](https://github.com/texane/stlink) dependency. This software collection contains a utility called `st-flash` which can be used to write .bin files to the microcontroller. There are blog posts and information online how to set this up like [this post](https://fishpepper.de/2016/09/16/installing-using-st-link-v2-to-flash-stm32-on-linux/).
2.  Download the `arm-none-eabi` from the [website](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) or use a package manager (e.g. `sudo apt install 
binutils-arm-none-eabi gcc-arm-none-eabi libnewlib-arm-none-eabi gdb-multiarch`).
3. Download `openocd`.
4.  To make it easy to set up the openocd server, make a simple bash script with the following line `` while true; do openocd -f /usr/local/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/local/share/openocd/scripts/target/stm32f1x.cfg ; sleep 1;done``. This keeps trying to set up an session (make sure those config files exist at these locations or change the absolute path).

At this point we should at least compile this project. Make sure you are in the project directory and try `make`.
if this went well, a number of files would have been created such as main.elf, main.bin and main.map. If an error occurred during make you will have to resolve it.

### Flash to device

We can use the command `st-flash write main.bin 0x08000000` to write main.bin at location 0x08000000 (here starts the flash according to the memory table described in the STM32F103C8T6 datasheet). 


### Debug using GDB

To debug you must execute the script just given in 4.  Now you have to open another terminal and use `arm-none-eabi-gdb` or `gdb-multiarch"`(whichever gdb debugger you have as long as it is **NOT** the regular x86 one).   Write the following three commands in the gdb session:

```
(gdb) target remote localhost:3333
(gdb) file main.elf
(gdb) load
```
* `target remote localhost:3333`: Tell gdb that we want to connect to a remote computer (the microcontroller in this case).
*  `file main.elf`: Tell gdb which program we actually want to debug
* `load`: Load the program to be debugged

Now we are able to set breakpoints, print variables and single step through our code :)

