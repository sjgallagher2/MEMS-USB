st-flash write MEMS-USB.binary 0x8000000
openocd -f stlink-v2.cfg -f stm32f4x.cfg &
arm-none-eabi-gdb MEMS-USB.elf
