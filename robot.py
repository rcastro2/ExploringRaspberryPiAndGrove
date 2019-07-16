from grovelib import *
from time import sleep

relay = Relay(5)
button = Button(22)

while True:
    print(button.is_pressed)
    if button.is_pressed:
        relay.on()
    else:
        relay.off()
    sleep(0.5)

    
