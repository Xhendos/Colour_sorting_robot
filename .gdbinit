set confirm off
target remote localhost:3333
file main.elf
load
break src/main.c:main
break src/octo/octo.c:init_task
break src/octo/octo.c:manager_task
break src/octo/octo.c:USART1_IRQ_handler
set confirm on
