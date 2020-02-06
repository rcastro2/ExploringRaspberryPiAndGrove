from __future__ import division

from grove.gpio import GPIO
from grove.adc import ADC
from grove.factory import Factory
from grove.button import Button
from grove.i2c import Bus

import RPi.GPIO as GPIORPi
def set_max_priority(): pass
def set_default_priority(): pass
GPIORPi.setmode(GPIORPi.BCM)
GPIORPi.setwarnings(False)
from time import sleep,time
from numpy import interp


class Buzzer(object):
    def __init__(self,pin):
        #Digital Port
        self.pin = pin
        GPIORPi.setup(self.pin, GPIORPi.OUT)
    def on(self):
        GPIORPi.output(self.pin, 1)
    def off(self):
        GPIORPi.output(self.pin, 0)

class Relay(GPIO):
    def __init__(self, pin):
        super(Relay, self).__init__(pin, GPIO.OUT)
        #Digital Port

    def on(self):
        self.write(1)

    def off(self):
        self.write(0)

class Led(GPIO):
    def __init__(self, pin):
        super(Led, self).__init__(pin, GPIO.OUT)
        #Digital Port
    def on(self):
        self.write(1)
        
    def off(self):
        self.write(0)

class Light(object):
    def __init__(self, pin):
        #Analog Port
        self.pin = pin
        self.adc = ADC()

    def read(self):
        value = self.adc.read(self.pin)
        return value

class Sound(object):
    def __init__(self, pin):
        #Analog Port
        self.pin = pin
        self.adc = ADC()

    def read(self):
        value = self.adc.read(self.pin)
        return value
    
class GSR(object):
    def __init__(self, pin):
        #Analog Port
        self.pin = pin
        self.adc = ADC()

    def read(self):
        value = self.adc.read(self.pin)
        return value
    
class Button(object):
    def __init__(self, pin):
        #Digital Port
        self.__btn = Factory.getButton("GPIO-HIGH", pin)

    @property
    def is_pressed(self):
        return self.__btn.is_pressed()

class Ultrasonic(object):
    def __init__(self, pin):
        #Digital Port
        self.dio = GPIO(pin)
        self._TIMEOUT1 = 1000
        self._TIMEOUT2 = 10000

    def usleep(self,x):
        sleep(x / 1000000.0)
        
    def _get_distance(self):
        self.dio.dir(GPIO.OUT)
        self.dio.write(0)
        self.usleep(2)
        self.dio.write(1)
        self.usleep(10)
        self.dio.write(0)

        self.dio.dir(GPIO.IN)

        t0 = time()
        count = 0
        while count < self._TIMEOUT1:
            if self.dio.read():
                break
            count += 1
        if count >= self._TIMEOUT1:
            return None

        t1 = time()
        count = 0
        while count < self._TIMEOUT2:
            if not self.dio.read():
                break
            count += 1
        if count >= self._TIMEOUT2:
            return None

        t2 = time()

        dt = int((t1 - t0) * 1000000)
        if dt > 530:
            return None

        distance = ((t2 - t1) * 1000000 / 29 / 2)    # cm

        return distance

    def read(self):
        while True:
            dist = self._get_distance()
            if dist:
                return dist

class Temperature(object):
    def __init__(self, pin):
        #Analog Port
        self.__tmp = Factory.getTemper("NTC-ADC", pin)

    def read(self, unit = 'c'):
        temp = self.__tmp.temperature
        if unit == 'f':
            temp = (temp * 9/5) + 32
        return temp



class DHT(object):
    def __init__(self, pin):
        #Digital Port
        self.pin = pin
        GPIORPi.setup(self.pin, GPIORPi.OUT)
        self._last_temp = 0.0
        self._last_humi = 0.0
        
    def _read(self):
        MAX_CNT = 320
        PULSES_CNT = 41
        # Send Falling signal to trigger sensor output data
        # Wait for 20ms to collect 42 bytes data
        GPIORPi.setup(self.pin, GPIORPi.OUT)
        set_max_priority()

        GPIORPi.output(self.pin, 1)
        sleep(.2)

        GPIORPi.output(self.pin, 0)
        sleep(.018)

        GPIORPi.setup(self.pin, GPIORPi.IN)
        # a short delay needed
        for i in range(10):
            pass

        # pullup by host 20-40 us
        count = 0
        while GPIORPi.input(self.pin):
            count += 1
            if count > MAX_CNT:
                # print("pullup by host 20-40us failed")
                set_default_priority()
                return None, "pullup by host 20-40us failed"

        pulse_cnt = [0] * (2 * PULSES_CNT)
        fix_crc = False
        for i in range(0, PULSES_CNT * 2, 2):
            while not GPIORPi.input(self.pin):
                pulse_cnt[i] += 1
                if pulse_cnt[i] > MAX_CNT:
                    # print("pulldown by DHT timeout %d" % i)
                    set_default_priority()
                    return None, "pulldown by DHT timeout %d" % i

            while GPIORPi.input(self.pin):
                pulse_cnt[i + 1] += 1
                if pulse_cnt[i + 1] > MAX_CNT:
                    # print("pullup by DHT timeout %d" % (i + 1))
                    if i == (PULSES_CNT - 1) * 2:
                        # fix_crc = True
                        # break
                        pass
                    set_default_priority()
                    return None, "pullup by DHT timeout %d" % i

        # back to normal priority
        set_default_priority()

        total_cnt = 0
        for i in range(2, 2 * PULSES_CNT, 2):
            total_cnt += pulse_cnt[i]

        # Low level ( 50 us) average counter
        average_cnt = total_cnt / (PULSES_CNT - 1)
        # print("low level average loop = %d" % average_cnt)
       
        data = ''
        for i in range(3, 2 * PULSES_CNT, 2):
            if pulse_cnt[i] > average_cnt:
                data += '1'
            else:
                data += '0'
        
        data0 = int(data[ 0: 8], 2)
        data1 = int(data[ 8:16], 2)
        data2 = int(data[16:24], 2)
        data3 = int(data[24:32], 2)
        data4 = int(data[32:40], 2)

        if fix_crc and data4 != ((data0 + data1 + data2 + data3) & 0xFF):
            data4 = data4 ^ 0x01
            data = data[0: PULSES_CNT - 2] + ('1' if data4 & 0x01 else '0')

        if data4 == ((data0 + data1 + data2 + data3) & 0xFF):
            humi = int(data0)
            temp = int(data2)
        else:
            # print("checksum error!")
            return None, "checksum error!"

        return humi, temp

    def read(self, retries = 15):
        for i in range(retries):
            humi, temp = self._read()
            if not humi is None:
                break
        if humi is None:
            return self._last_humi, self._last_temp
        self._last_humi,self._last_temp = humi, temp
        return humi, temp

class DCMotor(object):
    def __init__(self):
        self.speed = 0
        self.clockwise = True
        
class Motor(object):
	__MotorSpeedSet             = 0x82
	__PWMFrequenceSet           = 0x84
	__DirectionSet              = 0xaa
	__MotorSetA                 = 0xa1
	__MotorSetB                 = 0xa5
	__Nothing                   = 0x01
	__EnableStepper             = 0x1a
	__UnenableStepper           = 0x1b
	__Stepernu                  = 0x1c
	I2CAddr                     = 0x0f  #Set the address of the I2CMotorDriver
	SPEED_MAX                   = 100

	def __init__(self,address=0x0f):
                #I2C Port - Grove - I2C Motor Driver V1.3
		self.I2CAddr = address
		self.bus = Bus()
		self.motor = [null,DCMotor(),DCMotor]

	def __del__(self):
		self.set_speed(0, 0)

	#Maps speed from 0-100 to 0-255
	def _map_vals(self,value, leftMin, leftMax, rightMin, rightMax):
		#http://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another
		# Figure out how 'wide' each range is
		leftSpan = leftMax - leftMin
		rightSpan = rightMax - rightMin

		# Convert the left range into a 0-1 range (float)
		valueScaled = float(value - leftMin) / float(leftSpan)

		# Convert the 0-1 range into a value in the right range.
		return int(rightMin + (valueScaled * rightSpan))
	def update(self):
                self.set_dir(self.motor[1].clockwise, self.motor[2].clockwise)
                self.set_speed(self.motor[1].speed,self.motor[2].speed)
		
	#Set motor speed
	def set_speed(self, speed1 = 0, speed2 = 0):
		s1 = self._map_vals(speed1, 0, 100, 0, 255)
		s2 = self._map_vals(speed2, 0, 100, 0, 255)
		self.bus.write_i2c_block_data(self.I2CAddr, self.__MotorSpeedSet, [s1, s2])
		time.sleep(.02)
	
	#Set motor direction
	def set_dir(self, clock_wise1 = True, clock_wise2 = True):
		dir1 = 0b10 if clock_wise1 else 0b01
		dir2 = 0b10 if clock_wise2 else 0b01
		dir = (dir2 << 2) | dir1
		self.bus.write_i2c_block_data(self.I2CAddr, self.__DirectionSet, [dir, 0])
		time.sleep(.02)

class Servo(object):
        MIN_DEGREE = 0
        MAX_DEGREE = 180
        INIT_DUTY = 2.5

        def __init__(self, channel):
            #PWM Port
            GPIORPi.setup(channel,GPIORPi.OUT)
            self.pwm = GPIORPi.PWM(channel,50)
            self.pwm.start(Servo.INIT_DUTY)

        def __del__(self):
            self.pwm.stop()

        def setAngle(self, angle):
            # Map angle from range 0 ~ 180 to range 25 ~ 125
            angle = max(min(angle, Servo.MAX_DEGREE), Servo.MIN_DEGREE)
            tmp = interp(angle, [0, 180], [25, 125])
            self.pwm.ChangeDutyCycle(round(tmp/10.0, 1))
