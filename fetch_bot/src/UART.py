#!/usr/bin/python3
import time
import serial

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

	def motor_controls(self, forward_effort, turning_effort):
		self.m_serial_port.write(bytearray([85, 0, forward_effort, turning_effort + 100, 0, 0, 0, 0]))

	def servo_controls(self, actuated):
		self.m_serial_port.write(bytearray([85, 1, actuated, 0, 0, 0, 0, 0]))

	def IMU_request(self):
		self.m_serial_port.write(bytearray([85, 4, 0, 0, 0, 0, 0, 0]))
	
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

		self.m_accelx = 0 #m/s2
		self.m_accely = 0
		self.m_accelx = 0
		self.m_gyrox = 0 #deg/s
		self.m_gyroy = 0
		self.m_gyroz = 0

		self.G_SENSITIVITY = 8.75
		self.A_SENSITIVITY = .061
	
	def new_accelx(self, raw):
		self.m_accelx = raw * self.A_SENSITIVITY / 1000 * 9.81
	
	def new_accely(self, raw):
		self.m_accely = raw * self.A_SENSITIVITY / 1000 * 9.81

	def new_accelz(self, raw):
		self.m_accelz = raw * self.A_SENSITIVITY / 1000 * 9.81

	def new_gyrox(self, raw):
		self.m_gyrox = raw * self.G_SENSITIVITY / 1000

		print("GYRO X DATA IS " + str(self.m_gyrox))

	def new_gyroy(self, raw):
		self.m_gyroy = raw * self.G_SENSITIVITY / 1000

	def new_gyroz(self, raw):
		self.m_gyroz = raw * self.G_SENSITIVITY / 1000


if __name__ == "__main__":
	UART = UART()
	while True:
		UART.receive_data()