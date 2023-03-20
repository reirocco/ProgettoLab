from . import socketio, thread_lock, receiverThread, receiverThreadInterupt
from flask import session, request
from flask_socketio import SocketIO, emit, join_room, leave_room, \
    close_room, rooms, disconnect
from datetime import datetime, timedelta
from threading import Lock
from threading import Event
from flask import jsonify
import time
import json
import socket


def receiverThread(ip, port):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the port where the server is listening
    server_address = (ip, port)
    print('connecting to {} port {}'.format(*server_address))
    sock.connect(server_address)

    try:

        # Send data
        message = b'This is the message.  It will be repeated.'
        print('sending {!r}'.format(message))
        sock.sendall(message)

        # Look for the response
        amount_received = 0
        amount_expected = len(message)

        while amount_received < amount_expected:
            data = sock.recv(16)
            amount_received += len(data)
            print('received {!r}'.format(data))

    finally:
        print('closing socket')
        sock.close()


def start_thread_receiver(ip, port):
    global receiverThread
    global receiverThreadInterupt
    if receiverThread is None:
        with thread_lock:
            receiverThreadInterupt = None
            receiverThread = socketio.start_background_task(receiverThread, ip, port)
            print("activated")


start_thread_receiver("192.168.1.110",8888)
