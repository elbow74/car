import cv2
import time
from ultralytics import YOLO
import serial

def send_packet(left_speed, left_dir, right_speed, right_dir):
	packet = bytearray([
	ord('L'),
	left_speed,
	left_dir,
	ord('R'),
	right_speed,
	right_dir])
	ser.write(packet)


tracker = None
tracking = False

# default directions
STOP = [0,1,0,1]
FORWARD = [30, 0, 30, 0]
LEFT = [15, 0, 30, 0]
RIGHT = [30, 0, 15, 0]
ROTATE = [25, 1, 25, 0]
RGB_ON = [1, 1, 0, 1]
RGB_OFF = [1, 1, 0, 0]
BUZZER_TOGGLE = [1,1,1,1]
RGB = False

# Defining Serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
time.sleep(2)  # Give Arduino time to reset

# Load YOLOv8n model
model = YOLO("yolov8n.pt")

# Open USB webcam
cap = cv2.VideoCapture(0)  # Change this index if needed (0, 1, 2...)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 160)

if not cap.isOpened():
	print("Camera not connected")
	exit()

# Confirm webcam opens
ret, frame = cap.read()
frame = cv2.resize(frame, (320, 160))

if not ret:
	print("? Cannot open webcam.")
	exit()

# Get actual frame center based on resolution
frame_center_x = frame.shape[1] // 2  # width
frame_center_y = frame.shape[0] // 2  # height
print(frame.shape)
frame_count = 0
DETECT_EVERY = 240
last_results = []
start_time = time.time()
prev_dir= STOP
current_dir = FORWARD

error_x = 0
current_dir = STOP
prev_dir = 0

try:
	while True:

				
		ret, frame_old = cap.read()
		
		print("error_x: ", error_x)
		print("current_dir: ", current_dir)
		print("prev_dir   : ", prev_dir)
		
		frame = cv2.resize(frame_old, (320, 160))
		if not ret:
			print("?? Frame not received.")
			continue

		frame_count += 1
			


		# Run YOLO detection every N frames or if not -=tracking
		if frame_count % DETECT_EVERY == 0 or not tracking:

			# results obj (bounding boxes, class IDs, confidence)
			results = model(frame, verbose=False) # access with results[i].boxes/.conf/.cls
			last_results = results
			
			print("==================== Updated YOLO ====================")

			found = False
			for r in results:
				for box in r.boxes:
					cls = int(box.cls[0])
					conf = float(box.conf[0])
					if cls == 0 and conf > 0.50:  # Person class
						print("====================  FOUND SIGMA TARGET ====================")
						print("====================  FOUND SIGMA TARGET ====================")

						x1, y1, x2, y2 = map(int, box.xyxy[0])
						w, h = x2 - x1, y2 - y1
						tracker = cv2.legacy.TrackerMOSSE_create()
						tracker.init(frame, (x1, y1, w, h))
						tracking = True
						found = True
						break
					else:
						tracking = False
				if found:
					break

		# Track person
		if tracking and tracker is not None:
			print("====================  Attempting track using MOSSE ==================== ")
			success, box = tracker.update(frame)
			if success: 
				print("==================== Person found using MOSSE, following ==================== ")
				x, y, w, h = map(int, box)
				
				center_x = x + w // 2
				center_y = y + h // 2
				error_x = center_x - frame_center_x

				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
				cv2.putText(frame, f"Person {conf:.2f}", (x, y - 10),
							cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

				if error_x < -20:
					current_dir = LEFT
				elif error_x > 20:
					current_dir = RIGHT
				else:
					current_dir = FORWARD
				
				# Direction decision
				if prev_dir != current_dir:
					prev_dir = current_dir
					send_packet(*current_dir)
					print("==================== Sent new direction packet to motor ====================")
				
			else:
				print("==================== TRACKER LOST TARGET ====================")
				print("==================== TRACKER LOST TARGET ====================")
				print("==================== TRACKER LOST TARGET ====================")
				
				tracking = False
				send_packet(*STOP)
				current_dir = STOP
				
		# Draw center lines
		cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (255, 0, 0), 1)  # vertical
		cv2.line(frame, (0, frame_center_y), (frame.shape[1], frame_center_y), (255, 0, 0), 1)  # horizontal

		# FPS print
		elapsed = time.time() - start_time
		if elapsed >= 1.0:
			print(f"FPS: {frame_count / elapsed:.2f}")
			start_time = time.time()

		# Show the output
		cv2.imshow("Camera Output", frame)
		if cv2.waitKey(1) == ord('q'):
			break
			
except KeyboardInterrupt:
	send_packet(*STOP)
	cap.release()
	cv2.destroyAllWindows()
	print("Quit program")

