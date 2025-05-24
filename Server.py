import socket
import threading
from IR_Controller import handle_command

HOST = '0.0.0.0'
PORT = 5000

def handle_client(conn, addr):
    print(f"[CONNECTED] {addr}")
    try:
        data = conn.recv(1024).decode().strip()
        if data:
            print(f"[COMMAND] Received from {addr}: {data}")
            response = handle_command(data)
            conn.sendall(response.encode())
        else:
            conn.sendall(b"Invalid command.")
    except Exception as e:
        print(f"[ERROR] {e}")
        conn.sendall(b"Error occurred.")
    finally:
        conn.close()

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"[LISTENING] Server running on {HOST}:{PORT}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()

if __name__ == "__main__":
    start_server()
