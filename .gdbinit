set confirm off
target remote localhost:3333
file main.elf
load
define nn
set armId+=1
p/u armId
c
end
define pp
set armId-=1
p/u armId
c
end
