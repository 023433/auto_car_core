from .pwm import PWM

wheel = PWM(1, 0x5c)
wheel.setFreq(50)

GPIO_SERVO = 15

MIN_VECTOR  = 2.5
WITH_VECTOR = 10
steer_limit = 38
centerAngle = 90

value = 0
value *= -1

def _angle2duty(n):
  return (WITH_VECTOR / 180) * n + MIN_VECTOR

def turnLeft(angle):
  angle = (angle / 100) * -1
  angle = angle * steer_limit
  a = centerAngle - angle
  wheel.setDuty(GPIO_SERVO, _angle2duty(a))

def turnRight(angle):
  angle = (angle / 100) * -1
  angle = angle * steer_limit
  a = centerAngle + angle
  wheel.setDuty(GPIO_SERVO, _angle2duty(a))
