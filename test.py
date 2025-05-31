import socket
import json
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 10000

def send_data(sock, data):
    try:
        message = json.dumps(data).encode('utf-8')
        sock.sendto(message, (UDP_IP, UDP_PORT))
        print(f"Sent {len(message)} bytes")  # Debug output
    except Exception as e:
        print(f"Send error: {e}")

# Example usage
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
vehicle_data = {
    "frame": 1,
    "timestamp": time.time(),
    "position": {"x": 1.23, "y": 4.56, "z": 0.78},
    "rotation": {"yaw": 90.0, "pitch": 0.0, "roll": 0.0},
    "control": {"throttle": 0.75, "steer": 0.1, "brake": 0.0},
    "speed": 50.0
}
send_data(sock, vehicle_data)