import smbus2 as smbus

address=0x68
PW_MGMT_1 = 0x6b
PW_MGMT_2 = 0x6c

bus = smbus.SMBus(8)

bus.write_byte_data(address, PW_MGMT_1, 0)

def read_word(adr):
  high = bus.read_byte_data(address, adr)
  low = bus.read_byte_data(address, adr+1)
  val = (high << 8) + low
  return val

def read_word_2c(adr):
  val = read_word(adr)
  if (val >= 0x8000):
    return -((65535 - val) + 1)
  else:
    return val

def get_accel_x():
  return read_word_2c(0x3B)

def get_accel_y():
  return read_word_2c(0x3D)

def get_accel_z():
  return read_word_2c(0x3F)

def get_gyro_x():
  return read_word_2c(0x43)

def get_gyro_y():
  return read_word_2c(0x45)

def get_gyro_z():
  return read_word_2c(0x47)

def get_temp():
  return read_word_2c(0x41) / 340.00 + 36.53