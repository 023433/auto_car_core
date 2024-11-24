import time
import math
import smbus2 as smbus


class PWM(object):
  _mode_adr              = 0x00  # Mode 1 Reg
  _base_adr_low          = 0x08 
  _base_adr_high         = 0x09
  _prescale_adr          = 0xFE  # Prescale Reg

  def __init__(self, bus, address, wait_i2c=True):
    '''
    I2C 버스에 연결된 PWM 컨트롤러 인스턴스 생성
    @param bus: PWM 컨트롤러가 연결된 I2C 버스 번호(0, 1).
    @param address: PWM 컨트롤러 주소(기본값은 0x41)
    '''
    self.wait_i2c=wait_i2c
    self.bus = smbus.SMBus(bus)
    self.address = address
    self._writeByte(self._mode_adr, 0x00)

  def setFreq(self, freq):
    '''
    PWM 주파수 설정
    @param freq: Hz 단위 주파수
    '''
    prescaleValue = 25000000.0    # 25MHz
    prescaleValue /= 4096.0       # 12-bit
    prescaleValue /= float(freq)
    #prescaleValue -= 1.0
    prescale = math.floor(prescaleValue + 0.5)
    if prescale < 3:
        raise ValueError("주파수 설정 오류")

    oldmode = self._readByte(self._mode_adr)
    newmode = (oldmode & 0x7F) | 0x10 # mode 1, sleep
    self._writeByte(self._mode_adr, newmode)
    self._writeByte(self._prescale_adr, int(math.floor(prescale)))
    self._writeByte(self._mode_adr, oldmode)
    time.sleep(0.005)
    self._writeByte(self._mode_adr, oldmode | 0xA1) #mode 1, autoincrement on (old 0x80)

  def setDuty(self, channel, duty):
    '''
    PWM 채널의 듀티비 설정
    @param channel: 채널 번호 (0~15)
    @param duty: 튜티비 (0~100)
    '''
    data = int(duty * 4096 / 100) # 0..4096 (included)
    
    # if self.wait_i2c:
    #     while time.time()-__main__.pwm_time_log<0.05: time.sleep(0.01)
    self._writeByte(self._base_adr_low + 4 * channel, data & 0xFF)
    self._writeByte(self._base_adr_high + 4 * channel, data >> 8)

  def _writeByte(self, reg, value):
    try:
      self.bus.write_byte_data(self.address, reg, value)
    except Exception as e:
      v=ValueError("[Errno "+str(e.errno)+"] An error occured while reading I2C Devcie")
      v.errno=e.errno
      raise v

  def _readByte(self, reg):
    try:
      result = self.bus.read_byte_data(self.address, reg)
      return result
    except Exception as e:
      v=ValueError("[Errno "+str(e.errno)+"] An error occured while reading I2C Devcie")
      v.errno=e.errno
      raise v