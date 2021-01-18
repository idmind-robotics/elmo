import herkulex
from herkulex import servo

herkulex.connect("/dev/ttyAMA0", 115200)
herkulex.clear_errors()
s = servo(3)
l = servo(4)
s.get_servo_status()
l.get_servo_status()
s.torque_on()
s.get_servo_position()
s.set_servo_position(100, 50, 0x00)