set confirm off
target remote localhost:3333
file main.elf
load
break main.c:78
break main.c:99
