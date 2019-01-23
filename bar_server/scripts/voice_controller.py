#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 17 19:46:33 2018

@author: jakyoon
"""
import pyaudio
import wave
import base64, requests,os, json
import rospy
import sys
from ctypes import *
import chardet

# Define our error handler type
def voice_recognizer():
    # error handler
    ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
    def py_error_handler(filename, line, function, err, fmt):
        None
    c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
    asound = cdll.LoadLibrary('libasound.so')
    # Set error handler
    asound.snd_lib_error_set_handler(c_error_handler)

    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = 3
    WAVE_OUTPUT_FILENAME = "output.wav"

    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)
    print ('')
    print ('请说出您的指令（3s）')
    print("* recording")

    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("* done recording")

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    with open('output.wav', "rb") as f:
        # speech = base64.b64encode(f.read()).decode('utf-8')
        speech = base64.encodestring(f.read()).replace('\n', '')
    size = os.path.getsize('output.wav')
    data = {
        "format": "wav",
        "rate": 16000,
        "channel": 1,
        "token": "24.9a4f2319fe2e2025e4131d4410929454.2592000.1547670303.282335-10266003",
        "cuid": "1",
        "len": size,
        "speech" : speech
    }
    result = requests.post('http://vop.baidu.com/server_api', data=json.dumps(data), headers={'content-type': 'application/json'})
    data_result = result.json()
    if data_result['err_msg']=='success.':
        return data_result['result'][0].encode('utf-8')
    else:
        # print (data_result)
        return None

def get_command(data):
    keywords_to_command = {'hello': ['你好', '您好'],
            'whiskey': ['威士忌'],
            'beer': ['啤酒'],
            'shutdown': ['再见']
            }
    for (command, keywords) in keywords_to_command.iteritems():
        for word in keywords:
            if data.find(word) > -1:
                return command
    return None


def main(args):
    rospy.init_node('voice_controller')
    while not rospy.is_shutdown():
        sentence = voice_recognizer()
        if not sentence == None:
            print (sentence)
            cmd = get_command(sentence)
            print (get_command(sentence))
            if cmd == 'hello':
                pypath = 'rosrun bar_server hello.py'
            elif cmd == 'whiskey':
                pypath = 'rosrun bar_server cup_detector.py purple'
            elif cmd == 'beer':
                pypath = 'rosrun bar_server cup_detector.py orange'
            elif cmd == 'shutdown':
                break
            else:
                print ('指令无效')
                continue
            os.system(pypath)
        else:
            print ('我没有听清，请再说一遍')
 
if __name__ == '__main__':
    main(sys.argv)
 