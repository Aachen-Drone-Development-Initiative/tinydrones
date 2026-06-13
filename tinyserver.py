#!/usr/bin/env python3

import serial
import socket
import threading
import argparse
from message_interface.serial_packets import SPKT_MAX_SIZE

serialport = None
serialport_write_lock = None
clients: set[socket.socket] = set()
clients_lock = None

# Adress of the tinyserver
HOST = "127.0.0.1" # localhost (IPv4)
PORT = 6969


def serialport_read_and_broadcast_thread():
    while True:
        data = serialport.read(1)     # read_all() doesn't block so we block for at least one byte
        data += serialport.read_all() # and then read the rest
        
        with clients_lock:
            current_clients = list(clients)

        dead = []
        for client in current_clients:
            try:
                client.sendall(data)
            except OSError:
                dead.append(client)

        for client in dead:
            with clients_lock:
                clients.discard(client)
                client.close()


def client_read_thread(client: socket.socket, addr):
    try:
        while True:
            data = client.recv(SPKT_MAX_SIZE)
            if not data:
                break
            with serialport_write_lock:
                bytes_written = serialport.write(data)
                serialport.flush()
                if bytes_written != len(data):
                    raise RuntimeError(f"Failed to write all bytes: only {bytes_written} of {len(data)}")
    except OSError:
        pass
    finally:
        with clients_lock:
            clients.discard(client)
        client.close()
        print(f"INFO (MASTER): Client disconnected: {addr}")


def tcp_accept_thread():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows for fast reconnection of the same client
    server.bind((HOST, PORT))
    server.listen()
    print(f"INFO (MASTER): Listening on {HOST}:{PORT}")

    while True:
        client, addr = server.accept()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # Send data immediately, no buffering
        with clients_lock:
            clients.add(client)
        print(f"INFO (MASTER): Client connected: {addr}")

        thread = threading.Thread(target=client_read_thread,
                                  args=(client, addr),
                                  daemon=True) # Don't wait for this thread to finish before exiting
        thread.start()


def main():
    parser = argparse.ArgumentParser(description="tinyserver - 'multiplexer' for packets to/from the remote via serialport")
    parser.add_argument("--port", "-p")
    parser.add_argument("--baudrate", "-b")
    args = parser.parse_args()

    portname = args.port
    if portname == None:
        print("No serialport specified, use --port/-p <portname>")
        exit(1)
    baudrate = args.baudrate

    global serialport, serialport_write_lock, clients_lock
    serialport = serial.Serial(port=portname, baudrate=baudrate)
    serialport_write_lock = threading.Lock()
    clients_lock = threading.Lock()

    thread = threading.Thread(target=serialport_read_and_broadcast_thread,
                              daemon=True) # Don't wait for this thread to finish before exiting
    thread.start()
    tcp_accept_thread() # Run this on the main thread


if __name__ == "__main__":
    main()
