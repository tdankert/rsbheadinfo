#!/usr/bin/env python

import qi
import argparse
import sys
import numpy
import Image
from ImageChops import offset
import time
import collections


def postopixel(pos, offset, mpp, dims):
    return checkdims((int(pos[0] / mpp - offset[0] / mpp), int(-pos[1] / mpp + offset[1] / mpp)), dims)

def diftopixel(pos, mpp):
    return ((int(pos[0] / mpp), int(-pos[1] / mpp)))

def checkdims(px, dims):
    return (checkdim(px[0], dims[0], "x"), checkdim(px[1], dims[1], "y"))

def checkdim(px, dim, n):
    if px < 0:
        print("invalid " + str(n) + "-value: " + str(px) + " -> 0")
        px = 0
    elif px >= dim:
        print("invalid " + str(n) + "-value: " + str(px) + " -> " + str(dim - 1))
        px = dim - 1
    return int(px)


def draw_cross(buf, p, c, d):
    buf.putpixel(checkdims((p[0], p[1] - p[3]), d), c[1])
    buf.putpixel(checkdims((p[0] - p[2], p[1]), d), c[1])
    buf.putpixel(checkdims((p[0], p[1]), d), c[0])
    buf.putpixel(checkdims((p[0], p[1] + p[3]), d), c[1])
    buf.putpixel(checkdims((p[0] + p[2], p[1]), d), c[1])

def main(session):
    navigation_service = session.service("ALNavigation")
    result_map = navigation_service.getMetricalMap()
    mpp = result_map[0]
    map_width = result_map[1]
    map_height = result_map[2]
    orig_offset = result_map[3]
    map_data = result_map[4]
    dims = (int(map_width), int(map_height))

    print("meters per pixel: " + str(mpp))
    print("map width: " + str(map_width))
    print("map height: " + str(map_height))
    print("offset: " + str(orig_offset))
    print(orig_offset[0] / mpp)
    print(orig_offset[1] / mpp)
    img = numpy.array(map_data).reshape(map_width, map_height)
    img = (100 - img) * 2.55 # from 0..100 to 255..0
    img = numpy.array(img, numpy.uint8)
    disp = Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1)
    disp = disp.convert("RGB")
    hist = collections.deque(maxlen=30)


    intro = postopixel((-2.65, -1.4), orig_offset, mpp, dims)
    tumba = postopixel((-2.95, 0.76), orig_offset, mpp, dims)
    stein = postopixel((2.75, -0.66), orig_offset, mpp, dims)
    zerop = postopixel((0, 0), orig_offset, mpp, dims)

    intro = (intro[0], intro[1], 2, 2)
    tumba = (tumba[0], tumba[1], 2, 2)
    stein = (stein[0], stein[1], 2, 2)
    zerop = (zerop[0], zerop[1], 2, 2)

    scale = 5
    i = 0
    while True:
        robot_position = navigation_service.getRobotPositionInMap()
        rob = postopixel(robot_position[0], orig_offset, mpp, dims)
        unc = diftopixel(robot_position[1], mpp)
        rpos = (rob[0], rob[1], unc[0]/2, unc[1]/2)
        print(str(robot_position) + " -> " + str(rpos))

        if len(hist) == 0 or rpos != hist[len(hist) - 1]:
            hist.append(rpos)
        else:
            print("robot didn't move")

        buf = disp.copy()
        buf = buf.transpose(Image.FLIP_TOP_BOTTOM)
        buf = buf.transpose(Image.ROTATE_270)

        mid = (0, 0, 0)
        draw_cross(buf, intro, (mid, (255,0,0)), dims)
        draw_cross(buf, tumba, (mid, (0,0,255)), dims)
        draw_cross(buf, stein, (mid, (255,255,0)), dims)
        draw_cross(buf, zerop, (mid, (255,0,255)), dims)

        for i in range(len(hist)):
            r = hist[i]
            if i == len(hist) - 1:
                draw_cross(buf, (r[0], r[1], r[2]/2, r[3]/2), (mid, (0,255,0)), dims)
                draw_cross(buf, r, (mid, (180,255,180)), dims)
            else:
                buf.putpixel((r[0], r[1]), (20, 100, 20))

        #buf = buf.transpose(Image.FLIP_LEFT_RIGHT)
        #buf = buf.transpose(Image.ROTATE_270)

        buf = buf.resize((map_width * scale, map_height * scale))
        buf.save("/tmp/pos.jpg")
        time.sleep(1)
        i = i + 1
        i = i % 10


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="pepper.local",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
