set confirm off
target remote localhost:3333
file main.elf
load
define qq
p/u tx
c
p/u rx
c
end
define ee
p/u index
p/u goalPositions[index]
p/u presentPositions[index]
end
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
