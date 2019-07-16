from flask import Flask
from flask_socketio import SocketIO, emit
from threading import Thread
from grovelib import *
from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.resolution = (320,240)
camera.rotation = 180
 
app = Flask(__name__)
socketio = SocketIO(app)
class Robot:
    def __init__(self):
        self.command = "stop"
        self.speed = 50

r = Robot()
driver = Motor()
driver.motor[1].clockwise = False

@socketio.on('command')
def connect_handler(command):
    r.command = command
    print("Update: " + r.command)
    emit('robot', r.command, broadcast=True )

def robotloop():
    while True:
        if "forward" == r.command:
            driver.motor[1].clockwise = False
            driver.motor[2].clockwise = True
            driver.motor[1].speed = r.speed 
            driver.motor[2].speed = r.speed
        elif "left" == r.command:
            driver.motor[1].clockwise = True
            driver.motor[2].clockwise = True
            driver.motor[1].speed = r.speed * 0.75
            driver.motor[2].speed = r.speed * 0.75
        elif "right" == r.command:
            driver.motor[1].clockwise = False
            driver.motor[2].clockwise = False
            driver.motor[1].speed = r.speed * 0.75 
            driver.motor[2].speed = r.speed * 0.75
        elif "back" == r.command:
            driver.motor[1].clockwise = True
            driver.motor[2].clockwise = False
            driver.motor[1].speed = r.speed
            driver.motor[2].speed = r.speed
        else:
            driver.motor[1].speed = 0
            driver.motor[2].speed = 0
        driver.update()
        print("Status: " + r.command)
        camera.capture('/home/pi/Robotics/image.jpg')
        sleep(0.25)
        
if __name__ == '__main__':
    robotthread = Thread(target=robotloop,daemon=True)
    robotthread.start()
    socketio.run(app, port=5001, log_output=False, debug=False)
