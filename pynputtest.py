from pynput import keyboard

def on_press(key):
    print(f"Key pressed: {key}", flush=True)

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
