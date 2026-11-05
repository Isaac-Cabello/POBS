import socket
import serial
import time

HOST = "0.0.0.0"
PORT = 8765
SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200

def main():
    with serial.Serial(SERIAL_PORT, BAUD, timeout=1) as esp:
        time.sleep(2)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((HOST, PORT))
            server.listen(1)
            print(f"Waiting for connection on port {PORT}...", flush=True)
            while True:
                conn, addr = server.accept()
                print(f"Connected: {addr}", flush=True)
                with conn:
                    conn.settimeout(0.2)
                    buffer = b""
                    last_command = [0.0] * 4
                    while True:
                        try:
                            data = conn.recv(1024)
                            if not data:
                                break
                            buffer += data
                            while b"\n" in buffer:
                                line, buffer = buffer.split(b"\n", 1)
                                try:
                                    parts = line.decode("ascii").split(",")
                                    if len(parts) != 3 or parts[0] != "LED":
                                        continue
                                    index = int(parts[1])
                                    state = int(parts[2])
                                    if index not in range(4) or state not in (0, 1):
                                        continue
                                    esp.write(f"LED,{index},{state}\n".encode())
                                    last_command[index] = time.monotonic()
                                except (ValueError, UnicodeDecodeError):
                                    continue

                        except socket.timeout:
                            pass
                        now = time.monotonic()
                        for i in range(4):
                            if now - last_command[i] > 0.5:
                                esp.write(f"LED,{i},0\n".encode())
                    for i in range(4):
                        esp.write(f"LED,{i},0\n".encode())

if __name__ == "__main__":
    main()
