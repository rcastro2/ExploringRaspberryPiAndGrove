from grovelib import *

car = Motor()

print("Motor 1")
car.motor[1].speed = 100
car.update()
print("Sleep")
sleep(2)
print("Motor 2")
car.motor[1].speed = 0
car.motor[2].speed = 100
car.update()
print("Sleep")
sleep(2)
car.motor[2].speed = 0
car.update()
print("Done")


