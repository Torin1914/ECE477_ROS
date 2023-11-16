#!/usr/bin/env python3
import time
import serial
import rospy
from std_msgs.msg import Bool
from fetch_bot.msg import Drive
from fetch_bot.msg import IMU

pub = None

class UART:
	def __init__(self):
		self.m_serial_port = serial.Serial(
    		port="/dev/ttyTHS1",
    		baudrate=115200,
    		bytesize=serial.EIGHTBITS,
    		parity=serial.PARITY_NONE,
    		stopbits=serial.STOPBITS_ONE,
		)

		self.m_pos_data = PosData()

		# Wait a second to let the port initialize
		time.sleep(1)

	#SEND
	def motor_controls(self, data):
		self.m_serial_port.write(bytearray([85, 0, data.forward, data.rotation + 100, 0, 0, 0, 0]))

	def servo_controls(self, actuated):
		self.m_serial_port.write(bytearray([85, 1, actuated, 0, 0, 0, 0, 0]))

	def IMU_request(self):
		self.m_serial_port.write(bytearray([85, 4, 0, 0, 0, 0, 0, 0]))
	
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
			elif(ID == 7):
				self.accel_z(byte1, byte2, byte3, byte4)
			elif(ID == 8):
				self.gyro_x(byte1, byte2, byte3, byte4)
			elif(ID == 9):
				self.gyro_y(byte1, byte2, byte3, byte4)
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

		self.count = 0
	
	def new_accelx(self, raw):
		self.msg.accelx = raw * self.A_SENSITIVITY / 1000 * 9.81
		self.publish()
	
	def new_accely(self, raw):
		self.msg.accely = raw * self.A_SENSITIVITY / 1000 * 9.81
		self.publish()

	def new_gyroz(self, raw):
		self.msg.gyroz = raw * self.G_SENSITIVITY / 1000
		self.publish()

	def publish(self):
		global pub
		count += 1
		if count == 3:
			pub.publish(self.msg)
			count = 0

def fakeMotors(data: Drive):
	print(f"F: {data.forward}, R: {data.rotation}")

def fakeServo(data: bool):
	print(f"Close arms? {data}")

if __name__ == "__main__":
	# uart = UART()
	rospy.init_node("UART")
	# rospy.Subscriber("closeArmsUART", Bool, uart.servo_control)
	# rospy.Subscriber("drive", Drive, uart.motor_controls)
	rospy.Subscriber("closeArmsUART", Bool, fakeServo)
	rospy.Subscriber("drive", Drive, fakeMotors)

	pub = rospy.Publisher("uart2return2sender", IMU)
	msg = IMU()
	msg.accelx = 0
	msg.accely = 0
	msg.gyroz = 0

	while True:
		pub.publish(msg)
	# 	UART.receive_data()