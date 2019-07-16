
from grovelib import *

#button = Button(26) #Digital
#led = Led(22) #Digital
#led2 = Led(16)
#light = Light(0) #Analog
#dht11 = DHT(22) #Digital
#sound = Sound(2) #Analog
#temp = Temperature(6) #Analog
#ultra = Ultrasonic(24) #Digital
#buzzer = Buzzer(16) #Digital
#relay = Relay(18) #Digital
servo = Servo(12) #PWM
while True:
    
    #print(light.read())
    #print(dht11.read())
    #print(sound.read())
    #print(button.is_pressed)
    #print(temp.read(),temp.read('f'))
    #print(ultra.read())
    
    #led.light('on')
    #sleep(0.5)
    #led.light('off')
    #sleep(0.5)
    '''
    if light.read() < 100 or sound.read() > 600:
        led.light('on')
    else:
        led.light('off')
    '''
    '''
    if button.is_pressed :
        led.light('on')
        #led2.light('off')
        buzzer.play()
        #relay.on()
    else:
        led.light('off')
        #led2.light('on')
        buzzer.stop()
        #relay.off()      
    '''
    '''
    relay.on()
    sleep(1)
    relay.off()
    sleep(1)
    '''
    servo.setAngle(45)
    sleep(2)
    servo.setAngle(90)
    sleep(2)
    print("_______________")
    sleep(0.25)

    


    
