import serial
import time
import keyboard


def send_packet(packet):
    ser.write(packet.encode())
    print("Sent:", packet)

def read_available():
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print("Received:", data)

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)  # non-blocking mode
    time.sleep(2)  # Allow Arduino to reset

    try:
        while True:
            if keyboard.is_pressed('left'):
                send_packet("<LEFT,20,1,RIGHT,40,1>")
                print("left")
            elif keyboard.is_pressed('right'):
                send_packet("<LEFT,40,1,RIGHT,20,1>")
            elif keyboard.is_pressed('up'):
                send_packet("<LEFT,40,1,RIGHT,40,1>")
            elif keyboard.is_pressed('down'):
                send_packet("<LEFT,40,0,RIGHT,40,0>")
            elif keyboard.is_pressed('space'):
                send_packet("<LEFT,0,1,RIGHT,0,1>")
            elif keyboard.is_pressed('q'):
                print("Quitting...")
                break
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        ser.close()
