import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200
NUM_LEDS = 4

def send_command(esp, index, state):
    command = f"LED,{index},{state}\n"
    esp.write(command.encode("ascii"))
    esp.flush()
def main():
    print("Connecting to ESP32...")
    with serial.Serial(PORT, BAUD, timeout=1) as esp:
        time.sleep(2)
        print("ESP32 connected!")
        try:
            # Test each LED individually
            for i in range(NUM_LEDS):
                print(f"Testing LED {i}")
                # Keep LED on for 2 seconds
                for _ in range(20):
                    send_command(esp, i, 1)
                    time.sleep(0.1)
                send_command(esp, i, 0)
                print(f"LED {i} OFF")
                time.sleep(1)
        finally:
            # Turn all LEDs off
            for i in range(NUM_LEDS):
                send_command(esp, i, 0)
    print("All LED tests complete!")

if __name__ == "__main__":
    main()
