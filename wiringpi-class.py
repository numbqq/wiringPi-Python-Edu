%pythoncode %{
import serial
import smbus2
PWM_List = [0]*64

class Pin(object):
  IN = 0
  OUT = 1
  PWM = 1
  ANALOG = 5
  PWM_RANGE = 255
#  PWM_List = [0]*64
  def __init__(self,PinID=0,PinMode=0):
    if PinID == "D0" : Pin = 0
    elif PinID == "D1" : Pin = 1
    elif PinID == "D2" : Pin = 2
    elif PinID == "D3" : Pin = 3
    elif PinID == "D4" : Pin = 4
    elif PinID == "D5" : Pin = 5
    elif PinID == "D6" : Pin = 6
    elif PinID == "D7" : Pin = 7
#    elif PinID == "D8" : Pin = 8
    else : Pin=PinID
    wiringPiSetup()
    self.pin = Pin
    self.Mode = PinMode
#    pinMode(self.pin, self.Mode)
    if self.Mode == self.OUT or self.Mode == self.PWM :
      pinMode(self.pin, self.Mode)
      softPwmCreate(self.pin,0,self.PWM_RANGE)
      PWM_List[self.pin] = 1   # Mark that pin and initialize it to PWM
    elif self.Mode == self.ANALOG :
      if self.pin == "A0" or self.pin == "a0" :
        self.ADC_Pin = 17
      elif self.pin == "A1" or self.pin == "a1" :
        self.ADC_Pin = 18
    else :
      pinMode(self.pin, self.Mode)
  def __del__(self):
    PWM_List[self.pin] = 0
  def write_digital(self,level):
    if 1 == PWM_List[self.pin] :
      softPwmStop(self.pin)
    digitalWrite(self.pin,level)
  def read_digital(self):
    return digitalRead(self.pin)
  def write_analog(self,value=128):
    return softPwmWrite(self.pin,value)
  def read_analog(self):
    return analogRead(self.ADC_Pin)
    

class SerialMgt(object):
  device = '/dev/ttyS3'
  serial_id = 0
  def __init__(self,baudrate,timeout):
    self.timeout=timeout
    self.baudrate = baudrate
    self.serial_id = serial.Serial(port=self.device,baudrate=self.baudrate,timeout=self.timeout)
#    self.serial_id.open()
  def __del__(self):
    self.serial_id.close()
    del self.serial_id
  def write(self,bytes):
    return self.serial_id.write(bytes)
  def read(self,read_byte=1):
    return self.serial_id.read(read_byte)
  def readline(self):
    return self.serial_id.readline()

class Servo(object):
  PWM_RANGE = 180
  PWM = 1
  def __init__(self,PinID):
    wiringPiSetup()
    self.pin = PinID
    softPwmCreate(self.pin,0,self.PWM_RANGE)
  def __del__(self):
    PWM_List[self.pin] = 0 
  def write_angle(self,value):
    return softPwmWrite(self.pin,value)

class LED(object):
  Mode = 1
  def __init__(self,PinID):
    wiringPiSetup()
    self.pin = PinID
    pinMode(self.pin, self.Mode)
  def on(self):
    digitalWrite(self.pin,1)
  def off(self):
    digitalWrite(self.pin,0)
  def high(self):
    digitalWrite(self.pin,1)
  def low(self):
    digitalWrite(self.pin,0)

class I2C(object):
  port = 3
  def __init__(self):
    self.bus = smbus2.SMBus(self.port)
  def __del__(self):
    del self.bus
  def readfrom(self,address,register,read_bytes):
    return self.bus.read_i2c_block_data(address,register,read_bytes)
  def writeto(self,address,register,data):
    if register == None:
      self.bus.write_i2c_block_data(address,0,data)
    else :
      self.bus.write_i2c_block_data(address,register,data)
%}
