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



def receiverThread(ip, port):
    print("ciao")


def start_thread_receiver(ip, port):
    global receiverThread
    global receiverThreadInterupt
    global rediurl
    if receiverThread is None:
        with thread_lock:
            receiverThreadInterupt = None
            receiverThread = socketio.start_background_task(receiverThread,ip,port)
            print("activated")
