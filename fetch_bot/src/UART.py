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

		# Wait a second to let the port initialize
		time.sleep(1)

	def motor_controls(self, forward_effort, turning_effort):
		self.m_serial_port.write(bytearray([85,0,forward_effort,turning_effort+100,0,0,0,0]))

	def servo_controls(self, actuated):
		self.m_serial_port.write(bytearray([85,1,actuated,0,0,0,0,0]))

	def IMU_request(self):
		self.m_serial_port.write(bytearray([85,4,0,0,0,0,0,0]))

if __name__ == "__main__":
	UART = UART();
	while True:
		UART.IMU_request()

#try:
#    # Send a simple header
#    serial_port.write("UART Demonstration Program\r\n".encode())
#    serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())
#    while True:
#        if serial_port.inWaiting() > 0:
#            data = serial_port.read()
#            print(data)
#            serial_port.write(data)
#            # if we get a carriage return, add a line feed too
#            # \r is a carriage return; \n is a line feed
#            # This is to help the tty program on the other end 
#            # Windows is \r\n for carriage return, line feed
#            # Macintosh and Linux use \n
#            if data == "\r".encode():
#                # For Windows boxen on the other end
#                serial_port.write("\n".encode())


#except KeyboardInterrupt:
#    print("Exiting Program")

#except Exception as exception_error:
#    print("Error occurred. Exiting Program")
#    print("Error: " + str(exception_error))

#finally:
#    UART.m_serial_port.close()
#    pass
