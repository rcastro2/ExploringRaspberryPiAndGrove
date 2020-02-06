#grovelib is a library to facilitate the Raspberry Pi's interaction 
#with sensors and actuators

from grovelib import *
from flask import Flask

app = Flask(__name__)
lamp = Relay(5)

@app.route("/on")
def lampon():
    lamp.on()
    return "Light is on"

@app.route("/off")
def lampoff():
    lamp.off()
    return "Light is off"

@app.route("/")
def index():
    return "Welcome to Raspberry Pi"

@app.route("/message")
def message():
    return "Hello"
