#!/usr/bin/env python

import qi
import argparse
import sys
import numpy
import Image
import time
import collections

def main(session):

    navigation_service = session.service("ALNavigation")
    result_map = navigation_service.getMetricalMap()
    mpp = result_map[0]
    map_width = result_map[1]
    map_height = result_map[2]
    orig_offset = result_map[3]
    map_data = result_map[4]

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
    hist = collections.deque(maxlen=15)


    intro = (int(-1.5  / mpp - orig_offset[0] / mpp), int(-0.5  / mpp - orig_offset[1] / mpp))
    tumba = (int(1.5  / mpp - orig_offset[0] / mpp), int(0.2  / mpp - orig_offset[1] / mpp))
    stein = (int(1.5  / mpp - orig_offset[0] / mpp), int(-2.5  / mpp - orig_offset[1] / mpp))

    scale = 5
    i = 0
    while True:

        robot_position = navigation_service.getRobotPositionInMap()
        robot_x = int((robot_position[0][0] / mpp) - orig_offset[0] / mpp)
        robot_y = int((robot_position[0][1] / mpp) + orig_offset[1] / mpp)

        unc_x = int((robot_position[1][0] / mpp))
        unc_y = int((robot_position[1][1] / mpp))

        rpos=(robot_x, robot_y, unc_x, unc_y)
        print(str(robot_position) + " -> " + str(rpos))

        if len(hist) == 0 or rpos != hist[len(hist) - 1]:
            hist.append(rpos)
        else:
            print("robot didn't move")

        buf = disp.copy()
        buf.putpixel(intro, (255, 0, 0))
        buf.putpixel(tumba, (255, 0, 0))
        buf.putpixel(stein, (255, 0, 0))
        #buf = buf.transpose(Image.ROTATE_180)

        for i in range(len(hist)):
            r = hist[i]
            if i == len(hist) - 1:
                col = (0,255,0)
                mid = (0, 0, 0)

                buf.putpixel((r[0], max(0, min(map_height, r[1] - unc_y))), col)
                buf.putpixel((max(0, min(map_width, r[0] - unc_x)), r[1]), col)
                buf.putpixel((r[0], r[1]), mid)
                buf.putpixel((r[0], max(0, min(map_height, r[1] + unc_y))), col)
                buf.putpixel((max(0, min(map_width, r[0] + unc_x)), r[1]), col)
            else:
                buf.putpixel((r[0], r[1]), (20, 100, 20))


        buf = buf.resize((map_width * scale, map_height * scale))
        buf.save("/tmp/pos.jpg")
        time.sleep(2)
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
