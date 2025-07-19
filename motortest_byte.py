import os
os.environ["QT_QPA_PLATFORM"] = "xcb"  # Fix for Wayland error

import cv2
import time
import serial
import threading

# === Serial Setup ===
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
time.sleep(2)

# === Camera Setup ===
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 160)

if not cap.isOpened():
	print("? Camera not connected.")
	exit()

# === Send Packet Function ===
def send_packet(left_speed, left_dir, right_speed, right_dir):
	packet = bytearray([
		ord('L'),
		left_speed,
		left_dir,
		ord('R'),
		right_speed,
		right_dir
	])
	ser.write(packet)
	print(f"Sent: L,{left_speed},{left_dir},R,{right_speed},{right_dir}")

# === Serial Reading Function ===
def read_available():
	if ser.in_waiting > 0:
		data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
		print("Received:", data)

# === Background Thread for Camera Feed ===
def camera_loop():
	while running:
		ret, frame = cap.read()
		
		if ret:
			frame = cv2.resize(frame, (320, 160))
			
			cv2.imshow("Camera Output", frame)
			cv2.waitKey(1)  # Needed to update window
		else:
			print("?? Frame read failed.")
			break

# === Main ===
running = True
camera_thread = threading.Thread(target=camera_loop)
camera_thread.start()

print("Control with W (forward), A (left), S (stop), D (right), Z (reverse), mspeed (motor speed 0-100), Q (quit)")

motorSpeed = 50
tunedMotorSpeed = 50 - 3
motorTurnSpeed = int(motorSpeed * 0.5)

print(f"Current motor speed is: {motorSpeed}")
print(f"Tuned motor speed (RIGHT) is: {tunedMotorSpeed}")


try:
	while True:
		cmd = input(">>> ").strip().lower()
		parts = cmd.split()  # splits into ["mspeed", "50"]

		if cmd == "w":
			send_packet(motorSpeed, 0, tunedMotorSpeed, 0)
		elif cmd == "a":
			send_packet(motorTurnSpeed, 0, motorSpeed, 0)
		elif cmd == "d":
			send_packet(motorSpeed, 0, motorTurnSpeed, 0)
		elif cmd == "s":
			send_packet(0, 0, 0, 0)
		elif cmd == "z":
			send_packet(motorSpeed, 1, tunedMotorSpeed, 1)
		elif len(parts) >= 2 and parts[0] == "mspeed":
			try:
				if len(parts) == 3 and parts[1] == "tune":
					tunedMotorSpeed = int(parts[2])
					print(f"Tuned Speed set to {tunedMotorSpeed}")
					
				elif len(parts) == 2:
					tunedMotorSpeed = int(tunedMotorSpeed * (int(parts[1]) / motorSpeed))
					motorSpeed = int(parts[1])
					motorTurnSpeed = int(motorSpeed * 0.5)

					print(f"Motor speed set to {motorSpeed}")
			except ValueError:
				print("?? Invalid speed. Please enter a number.")
		
		elif cmd == "b":
			send_packet(1, 1, 1, 1)
			
		elif cmd == "rgb":
			send_packet(1, 1, 0, 1)
			
		elif cmd == "q":
			print("Quitting...")
			break
		else:
			print("Unknown key. Use: W A S D Z Q")

		read_available()

except KeyboardInterrupt:
	print("Stopped by user.")

finally:
	running = False
	camera_thread.join()
	send_packet(0, 1, 0, 1)
	cap.release()
	cv2.destroyAllWindows()
	ser.close()
	print("Shutdown complete.")
