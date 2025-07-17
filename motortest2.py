import serial
import time

def send_packet(packet):
    ser.write(packet.encode())
    print("Sent:", packet)

def read_available():
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print("Received:", data)

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
    time.sleep(2)  # Give Arduino time to reset

    print("Control with W (forward), A (left), S (stop), D (right), Q (quit)")

    try:
        while True:
            cmd = input(">>> ").strip().lower()

            if cmd == "a":
                send_packet("<LEFT,5,1,RIGHT,15,1>")   # Turn left
            elif cmd == "d":
                send_packet("<LEFT,15,1,RIGHT,5,1>")   # Turn right
            elif cmd == "w":
                send_packet("<LEFT,15,1,RIGHT,15,1>")   # Forward
            elif cmd == "s":
                send_packet("<LEFT,0,1,RIGHT,0,1>")     # Stop
            elif cmd == "q":
                print("Quitting...")
                break
            else:
                print("Unknown key. Use: W A S D Q")

            read_available()
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        ser.close()
