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
for img in range(10000):
    photoCaptureProxy.takePicture("/tmp/", "current")
    if img % 100 == 0:
        end = time.time()
        print("took 100 pictures in " + (end - start) + "s")
        start = end
