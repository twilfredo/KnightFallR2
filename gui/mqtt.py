#!/usr/bin/env python3
import urllib.request
import requests
import threading
import json
import webbrowser
import threading
import time

import paho.mqtt.publish as publish
import psutil
import string


channel_ID = "channels/1501295/subscribe"
mqtt_host = "mqtt3.thingspeak.com"

mqtt_client_ID = "Kg0AGjE1JB0SLwwXFT0FJyQ"
mqtt_username = "Kg0AGjE1JB0SLwwXFT0FJyQ"
mqtt_password = "ULXFifqohlslHz9lU3Q/mntL"


t_transport = "websockets"
t_port = 80


topic = "channels/" + channel_ID + "/publish"
payload = "field8=" + str(88)


def t_pub():
    while(1):
        print("Writing Payload = ", payload, " to host: ", mqtt_host, " clientID= ",
              mqtt_client_ID, " User ", mqtt_username, " PWD ", mqtt_password)

        err = publish.single(topic, payload, hostname=mqtt_host, transport=t_transport, port=t_port,
                             client_id=mqtt_client_ID, auth={'username': mqtt_username, 'password': mqtt_password})
        print(err)
        time.sleep(5)


def t_sub():
    while(1):
        time.sleep(1)

    # Set and start threads
thread_pub = threading.Thread(target=t_pub, args=())
thread_sub = threading.Thread(target=t_sub, args=())

thread_pub.start()
thread_sub.start()
