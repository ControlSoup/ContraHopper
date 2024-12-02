import socket
import os
import logging 
import threading
import time


FILE_PATH = os.path.dirname(__file__)

LOG_PATH = os.path.join(FILE_PATH, 'logs/reciever.log')

if not os.path.exists(os.path.dirname(LOG_PATH)): 
    os.makedirs(os.path.dirname(LOG_PATH))

try:
    os.remove(LOG_PATH)
except FileNotFoundError:
    pass

logging.basicConfig(filename=LOG_PATH, filemode='w', level=logging.INFO)

class KeyboardThread(threading.Thread):

    def __init__(self, server_ref: socket.socket, input_cbk, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        self.server = server_ref
        super(KeyboardThread, self).__init__(name=name, daemon=True)
        self.start()

    def run(self):
        while True:
            self.input_cbk(self.server, input()) #waits to get input + Return

def callback(server, input):

    def server_send(string: str):
        server.send(string.encode())

    inputs = {
        'RE-CONNECT': server_send('Established Connection'),
        'START': server_send('STR'),
        'STOP': server_send('STP'),
        'ABORT': server_send('ABT'),
        'TERMINATE': server_send('!!!')
    }

    if (input in inputs):
        # Send a command at 20times at 100hz
        for _ in range(20):
            inputs[input] 
            time.sleep(0.01)
    else:
        print(f'Input is not possible: {input}')



if __name__ == "__main__":

    UDP_IP = "192.168.0.88" 
    SHARED_UDP_PORT = 4210
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.connect((UDP_IP, SHARED_UDP_PORT))

    kthread = KeyboardThread(server, callback)

    while True:
        data = server.recv(2048).decode()
        logging.log(logging.INFO, data)