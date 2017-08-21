#!/usr/bin/env python

import os
import sys
import time
from naoqi import ALProxy

IP = "127.0.0.1"
PORT = 9559

try:
  photoCaptureProxy = ALProxy("ALPhotoCapture", IP, PORT)
except Exception, e:
  print "Error when creating ALPhotoCapture proxy:"
  print str(e)
  exit(1)

print("configuring camera")
photoCaptureProxy.setResolution(2)
photoCaptureProxy.setPictureFormat("jpg")
photoCaptureProxy.halfPress()

print("reading images")
start = time.time()
img = 0
while True:
    photoCaptureProxy.takePicture("/tmp/", "current")
    img += 1
    if img % 100 == 0:
        end = time.time()
        dur = end - start
        fps = 100/dur
        print("took 100 pictures in " + str(dur) + "s (" + str(fps) + "fps)")
        start = end
