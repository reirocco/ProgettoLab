# -*- encoding: utf-8 -*-
"""
Copyright (c) 2019 - present AppSeed.us
"""
import socket
import sys
import time
from datetime import datetime
from random import random

from apps.home import blueprint
from flask import render_template, request, Flask
from flask_login import login_required
from flask_socketio import send, emit
from jinja2 import TemplateNotFound
from threading import Lock
from flask_socketio import SocketIO
from .. import socketio, thread_lock, thread




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



def test():
    while 1:
        print("ciao",file=sys.stderr)
        emit('test', data="{'msg': 'ok'}", broadcast=True)
        time.sleep(1)

@blueprint.route('/index')
@login_required
def index():
    global thread
    with thread_lock:
        if thread is None:
            #thread = socketio.start_background_task(receiverThread, "192.168.92.59", 8888)
            thread = socketio.start_background_task(test)

    return render_template('home/index.html', segment='index')


@blueprint.route('/<template>')
@login_required
def route_template(template):
    try:

        if not template.endswith('.html'):
            template += '.html'

        # Detect the current page
        segment = get_segment(request)

        # Serve the file (if exists) from app/templates/home/FILE.html
        return render_template("home/" + template, segment=segment)

    except TemplateNotFound:
        return render_template('home/page-404.html'), 404

    except:
        return render_template('home/page-500.html'), 500


# Helper - Extract current page name from request
def get_segment(request):
    try:

        segment = request.path.split('/')[-1]

        if segment == '':
            segment = 'index'

        return segment

    except:
        return None
