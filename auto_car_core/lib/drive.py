from .pwm import PWM

car = PWM(1, 0x5e)
car.setFreq(200)

GPIO_LEFT_FORWARD    = 0
GPIO_LEFT_BACKWARD   = 1
GPIO_RIGHT_FORWARD   = 2
GPIO_RIGHT_BACKWARD  = 3
MIN_SPEED = 20
MAX_SPEED = 99


def stop():
  car.setDuty(GPIO_RIGHT_BACKWARD, 0)
  car.setDuty(GPIO_LEFT_FORWARD, 0)
  car.setDuty(GPIO_RIGHT_FORWARD, 0)
  car.setDuty(GPIO_LEFT_BACKWARD, 0)
  # print(f'stop')

def forward(speed):
  if speed == 0:
    stop()
    return
  
  rSpeed = limit(speed)
  car.setDuty(GPIO_RIGHT_BACKWARD, rSpeed)
  car.setDuty(GPIO_LEFT_FORWARD, rSpeed)
  car.setDuty(GPIO_RIGHT_FORWARD, 0)
  car.setDuty(GPIO_LEFT_BACKWARD, 0)
  # print(f'forward: {rSpeed}')

def backward(speed):
  if speed == 0:
    stop()
    return

  rSpeed = limit(speed)
  car.setDuty(GPIO_RIGHT_BACKWARD, 0)
  car.setDuty(GPIO_LEFT_FORWARD, 0)
  car.setDuty(GPIO_RIGHT_FORWARD, rSpeed)
  car.setDuty(GPIO_LEFT_BACKWARD, rSpeed)
  # print(f'backward: {rSpeed}')


def limit(speed):
  if speed < MIN_SPEED:
    return MIN_SPEED + speed
  
  if speed > MAX_SPEED:
    return MAX_SPEED
  
  return speed
  
