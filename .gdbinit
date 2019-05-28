set confirm off
target remote localhost:3333
file main.elf
load
break src/main.c:main
break src/octo/octo.c:USART1_IRQ_handler
break src/uart/uart.c:uart_send_byte
set confirm on
