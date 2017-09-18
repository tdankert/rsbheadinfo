#!/bin/bash
#gst-launch-0.10 multifilesrc caps=image/jpeg,framerate=20/1 location=/tmp/current.jpg loop=true ! decodebin ! jpegenc ! rtpjpegpay ! udpsink host=pepper port=3000
gst-launch-0.10 multifilesrc location=/tmp/current.jpg loop=true ! decodebin ! jpegenc ! rtpjpegpay ! udpsink host=pepper port=3000
