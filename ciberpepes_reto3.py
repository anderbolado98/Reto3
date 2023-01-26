 #!/usr/bin/env python
'''
Pymodbus Server With Updating Thread
--------------------------------------------------------------------------

This is an example of having a background thread updating the
context while the server is operating. This can also be done with
a python thread::

    from threading import Thread

    thread = Thread(target=updating_writer, args=(context,))
    thread.start()
'''
#---------------------------------------------------------------------------# 
# import the modbus libraries we need
#---------------------------------------------------------------------------# 

from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

#---------------------------------------------------------------------------# 
# import the twisted libraries we need
#---------------------------------------------------------------------------# 
from twisted.internet.task import LoopingCall

#---------------------------------------------------------------------------# 
# import wiringPi
#---------------------------------------------------------------------------# 
import wiringpi

#---------------------------------------------------------------------------# 
# configure the service logging
#---------------------------------------------------------------------------# 
#import logging
#logging.basicConfig()
#log = logging.getLogger()
#log.setLevel(logging.DEBUG)

#---------------------------------------------------------------------------# 
# DHT sensor configuration
#---------------------------------------------------------------------------#
import sys
import Adafruit_DHT
import RPi.GPIO as GPIO

sensor = Adafruit_DHT.DHT11
pin = 4
led_pin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.output(led_pin, GPIO.LOW)
#---------------------------------------------------------------------------# 
# Threading
#---------------------------------------------------------------------------#
from threading import Thread
from time import sleep
threadDelay = 2


#---------------------------------------------------------------------------# 
# DHT11 callback process
#---------------------------------------------------------------------------# 
import datetime

def updating_writer_DHT11():
    ''' A worker process that runs every so often and
    updates live values of the context. It should be noted
    that there is a race condition for the update.

    :param arguments: The input arguments to the call
    '''
 
    global humidity, temperature
    while 1:
	humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
	    
	# Note that sometimes you won't get a reading and
	# the results will be null (because Linux can't
	# guarantee the timing of calls to read the sensor).  
	# If this happens try again!
	if humidity is None or temperature is None:
	    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
	
	if humidity is None or temperature is None:
	    humidity, temperature = (0, 0)

	if temperature >= 18:
		GPIO.output(led_pin, GPIO.HIGH)
	else:
		GPIO.output(led_pin, GPIO.LOW)

	string = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
	print string + ': Temp={0:0.1f}*  Humidity={1:0.1f}%'.format(temperature, humidity)        
        
	#delay
	sleep(threadDelay)
	GPIO.output(led_pin, GPIO.LOW)
#---------------------------------------------------------------------------# 
# Heartbit callback process
#---------------------------------------------------------------------------# 
def updating_writer_Heartbit():
    ''' A worker process that runs every so often and
    updates live values of the context. It should be noted
    that there is a race condition for the update.

    :param arguments: The input arguments to the call
    '''
    global heartbit
    while 1:

	heartbit += 1
	#delay 1 seconds
	sleep(1)

#---------------------------------------------------------------------------# 
# GPIO callback process
#---------------------------------------------------------------------------# 
io = wiringpi.GPIO(wiringpi.GPIO.WPI_MODE_PINS)
io.pinMode(1, io.INPUT)
io.pinMode(2, io.INPUT)
io.pinMode(3, io.OUTPUT)
io.pinMode(4, io.OUTPUT)

def updating_writer_GPIO():

    global register_GPIO_INPUT, register_GPIO_OUTPUT

    while 1:
        #GPIO Reading
        
        #GPIO 1 - reading - SWITCH 1
        if io.digitalRead(1) == io.HIGH:
            register_GPIO_INPUT |= 0x01
        else:
            register_GPIO_INPUT &= 0xFE

        #GPIO 2 - reading - SWITCH 2
        if io.digitalRead(2) == io.HIGH:
            register_GPIO_INPUT |= 0x02
        else:
            register_GPIO_INPUT &= 0xFD

        #GPIO Writing
         
        #GPIO 3 - writing - LED
        if register_GPIO_OUTPUT & 0x01:
            io.digitalWrite(3, io.HIGH)
        else:
            io.digitalWrite(3, io.LOW)

        #GPIO 4 - writing - RELAY (control by GND)
        if register_GPIO_OUTPUT & 0x02:
            io.digitalWrite(4, io.LOW)
        else:
            io.digitalWrite(4, io.HIGH)

        sleep(0.5)

#---------------------------------------------------------------------------# 
# callback process
#---------------------------------------------------------------------------# 
def updating_writer(a):
    ''' A worker process that runs every so often and
    updates live values of the context. It should be noted
    that there is a race condition for the update.

    :param arguments: The input arguments to the call
    '''

    global humidity, temperature, heartbit
    global register_GPIO_INPUT, register_GPIO_OUTPUT
	
    #log.debug("updating the context")
    context  = a[0]
    register = 3
    slave_id = 0x00
    address  = 0x00
    values   = context[slave_id].getValues(register, address, count=5)
    
    values[0] = heartbit
    values[1] = temperature
    values[2] = humidity
   
    #GPIO - reading
    values[3] = register_GPIO_INPUT

    #GPIO - writing
    register_GPIO_OUTPUT = values[4]

    #log.debug("new values: " + str(values))
    context[slave_id].setValues(register, address, values)

#---------------------------------------------------------------------------# 
# initialize your data store
#---------------------------------------------------------------------------# 
store = ModbusSlaveContext(
    di = ModbusSequentialDataBlock(0, [1]*100),
    co = ModbusSequentialDataBlock(0, [2]*100),
    hr = ModbusSequentialDataBlock(0, [3]*100),
    ir = ModbusSequentialDataBlock(0, [4]*100))
context = ModbusServerContext(slaves=store, single=True)

temperature = 0
humidity = 0
heartbit = 0
register_GPIO_INPUT = 0x00
register_GPIO_OUTPUT = 0x02

#---------------------------------------------------------------------------# 
# initialize the server information
#---------------------------------------------------------------------------# 
identity = ModbusDeviceIdentification()
identity.VendorName  = 'pymodbus'
identity.ProductCode = 'PM'
identity.VendorUrl   = 'http://github.com/bashwork/pymodbus/'
identity.ProductName = 'pymodbus Server'
identity.ModelName   = 'pymodbus Server'
identity.MajorMinorRevision = '1.0'

#---------------------------------------------------------------------------# 
# DHT11 thread
#---------------------------------------------------------------------------# 
thread_DHT11 = Thread(target=updating_writer_DHT11, args=())
thread_DHT11.daemon = True
thread_DHT11.start()

#---------------------------------------------------------------------------# 
# Heartbit thread
#---------------------------------------------------------------------------# 
thread_Heartbit = Thread(target=updating_writer_Heartbit, args=())
thread_Heartbit.daemon = True
thread_Heartbit.start()

#---------------------------------------------------------------------------# 
# GPIO thread
#---------------------------------------------------------------------------# 
thread_GPIO = Thread(target=updating_writer_GPIO, args=())
thread_GPIO.daemon = True
thread_GPIO.start()

#---------------------------------------------------------------------------# 
# Main loop
#---------------------------------------------------------------------------# 
time = 0.5 # 3 seconds delay
loop = LoopingCall(f=updating_writer, a=(context,))
loop.start(time, now=False) # initially delay by time

#---------------------------------------------------------------------------# 
# start the Modbus server
#---------------------------------------------------------------------------# 
StartTcpServer(context, identity=identity, address=("192.168.10.20", 502))

import atexit
atexit.register(lambda: GPIO.output(led_pin, GPIO.LOW))
