set confirm off
target remote localhost:3333
file main.elf
load
define pp
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
