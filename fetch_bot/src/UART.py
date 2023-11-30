#!/usr/bin/env python3
import time
import serial
import rospy
import signal
import math as m
from functools import partial
from std_msgs.msg import Bool
from fetch_bot.msg import Drive
from fetch_bot.msg import IMU

pub = None

class UART:
	def __init__(self):
		self.m_serial_port = serial.Serial(
    		port="/dev/ttyTHS1",
    		baudrate=230400,
    		bytesize=serial.EIGHTBITS,
    		parity=serial.PARITY_NONE,
    		stopbits=serial.STOPBITS_ONE,
		)

		self.m_pos_data = PosData()

		# Wait a second to let the port initialize
		time.sleep(1)

	#SEND
	def motor_controls(self, data):
		self.m_serial_port.write(bytearray([85, 0, data.forward + 100, data.rotation + 100, 0, 0, 0, 0]))

	def servo_controls(self, actuated):
		actuated = 1 if actuated else 0
		self.m_serial_port.write(bytearray([85, 1, actuated, 0, 0, 0, 0, 0]))

	def IMU_request(self):
		self.m_serial_port.write(bytearray([85, 4, 0, 0, 0, 0, 0, 0]))

	def return_ball(self, data: Drive):
		self.motor_controls(data)
	
	#RECEIVE
	def invalid_message(self):
		print("INVALID MESSAGE")
	
	def accel_x(self, byte1, byte2, byte3, byte4):
		self.m_pos_data.new_accelx(int.from_bytes(byte1 + byte2 + byte3 + byte4, byteorder='little', signed=True))

	def accel_y(self, byte1, byte2, byte3, byte4):
		self.m_pos_data.new_accely(int.from_bytes(byte1 + byte2 + byte3 + byte4, byteorder='little', signed=True))

	def accel_z(self, byte1, byte2, byte3, byte4):
		self.m_pos_data.new_accelz(int.from_bytes(byte1 + byte2 + byte3 + byte4, byteorder='little', signed=True))
		
	def gyro_x(self, byte1, byte2, byte3, byte4):
		self.m_pos_data.new_gyrox(int.from_bytes(byte1 + byte2 + byte3 + byte4, byteorder='little', signed=True))
		
	def gyro_y(self, byte1, byte2, byte3, byte4):
		self.m_pos_data.new_gyroy(int.from_bytes(byte1 + byte2 + byte3 + byte4, byteorder='little', signed=True))
		
	def gyro_z(self, byte1, byte2, byte3, byte4):
		self.m_pos_data.new_gyroz(int.from_bytes(byte1 + byte2 + byte3 + byte4, byteorder='little', signed=True))

	def receive_data(self):
		byte = self.m_serial_port.read()
		if(byte == b'U'):
			ID = int.from_bytes(self.m_serial_port.read(), "big")
			byte1 = self.m_serial_port.read()
			byte2 = self.m_serial_port.read()
			byte3 = self.m_serial_port.read()
			byte4 = self.m_serial_port.read()
			if(ID == 248 or ID == 247):
				self.invalid_message()
			elif(ID == 5):
				self.accel_x(byte1, byte2, byte3, byte4)
			elif(ID == 6):
				self.accel_y(byte1, byte2, byte3, byte4)
			# elif(ID == 7):
			# 	self.accel_z(byte1, byte2, byte3, byte4)
			# elif(ID == 8):
			# 	self.gyro_x(byte1, byte2, byte3, byte4)
			# elif(ID == 9):
			# 	self.gyro_y(byte1, byte2, byte3, byte4)
			elif(ID == 10):
				self.gyro_z(byte1, byte2, byte3, byte4)
			
class PosData:
	def __init__(self):
		self.msg = IMU()
		self.msg.accelx = 0 #m/s2
		self.msg.accely = 0
		self.msg.gyroz = 0 #deg/s

		self.G_SENSITIVITY = 8.75
		self.A_SENSITIVITY = .061

		self.accelx_adjust = 0
		self.accely_adjust = 0
		self.gyroz_adjust = 0
		self.mean_accelx = 0
		self.mean_accely = 0
		self.mean_gyroz = 0

		self.meanx_count = 0
		self.meany_count = 0
		self.meanz_count = 0

		self.count = 0
	
	def new_accelx(self, raw):
		self.msg.accelx = (raw - self.accelx_adjust) * self.A_SENSITIVITY / 1000 * 9.81
		if(self.meanx_count > 300):
			self.publish()
		elif(self.meanx_count == 300):
			self.meanx_count += 1
			self.mean_accelx += self.msg.accelx
			self.mean_accelx /= 300
			self.accelx_adust = self.mean_accelx
		else:
			self.meanx_count += 1
			self.mean_accelx += self.msg.accelx
	
	def new_accely(self, raw):
		self.msg.accely = (raw - self.accely_adjust) * self.A_SENSITIVITY / 1000 * 9.81
		if(self.meany_count > 300):
			self.publish()
		elif(self.meany_count == 300):
			self.meany_count += 1
			self.mean_accely += self.msg.accely
			self.mean_accely /= 300
			self.accely_adust = self.mean_accely
		else:
			self.meany_count += 1
			self.mean_accely += self.msg.accely

	def new_gyroz(self, raw):
		self.msg.gyroz = (raw - self.gyroz_adjust) * self.A_SENSITIVITY / 1000 * 9.81
		if(self.meanz_count > 300):
			self.publish()
		elif(self.meanz_count == 300):
			self.meanz_count += 1
			self.mean_gyroz += self.msg.gyroz
			self.mean_gyroz /= 300
			self.gyroz_adust = self.mean_gyroz
		else:
			self.meanz_count += 1
			self.mean_gyroz += self.msg.gyroz

	def publish(self):
		global pub
		self.count += 1
		if self.count == 3:
			pub.publish(self.msg)
			self.count = 0

def shutdown(signum, frame, uart2: UART):
	msg = Drive()
	msg.forward = 0
	msg.rotation = 0
	uart2.motor_controls(msg)
	uart2.servo_controls(0)

def aEffort2aVel(aEffort: int):
    return 0.11 * aEffort - 1.22

def start_spin(uart2: UART):
	msg = Drive()
	msg.rotation = 75
	uart2.motor_controls(msg)

	aVel = aEffort2aVel(msg.rotation) * m.pi / 180
	time = abs(2*m.pi / aVel)
	rospy.sleep(time)
	msg.rotation = 0
	uart2.motor_controls(msg)

if __name__ == "__main__":
	uart = UART()
	shutdown2 = partial(shutdown, uart2=uart)
	signal.signal(signal.SIGINT, shutdown2)
	rospy.init_node("UART")

	# start_spin(uart)

	rospy.Subscriber("closeArmsUART", Bool, uart.servo_controls, queue_size=10)
	rospy.Subscriber("drive", Drive, uart.motor_controls, queue_size=10)
	rospy.Subscriber("driveFinal", Drive, uart.return_ball, queue_size=10)

	pub = rospy.Publisher("imu", IMU, queue_size=10)
	# t = time.time()
	# flag = True
	while not rospy.is_shutdown():
		# t2 = time.time()
		# if t2 - t >= 1:
		# 	if flag:
		# 		uart.servo_controls(True)
		# 		flag = False
		# 	else:
		# 		uart.servo_controls(False)
		# 		flag = True
		# 	print(f"sent {flag}")
		# 	t = t2
		uart.receive_data()
