#grovelib is a library to facilitate the Raspberry Pi's interaction 
#with sensors and actuators

from grovelib import *
from flask import Flask, render_template

app = Flask(__name__)
motor = [Relay(5),Relay(16)]

@app.route("/")
def index():
    return render_template("controls.html")

@app.route("/forward")
def forward():
    motor[0].on()
    motor[1].on()
    return "Moving Forward"

@app.route("/left")
def left():
    motor[0].on()
    motor[1].off()
    return "Turning Left"

@app.route("/right")
def right():
    motor[0].off()
    motor[1].on()
    return "Turning Left"

@app.route("/stop")
def stop():
    motor[0].off()
    motor[1].off()
    return "Car Stopped"


