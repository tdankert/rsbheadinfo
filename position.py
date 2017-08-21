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
    hist = collections.deque(maxlen=10)

    scale = 5
    i = 0
    while True:

        robot_position = navigation_service.getRobotPositionInMap()
        robot_x = int((robot_position[0][0] / mpp) - orig_offset[0] / mpp)
        robot_y = int((robot_position[0][1] / mpp) + orig_offset[1] / mpp)

        rpos=(robot_x, robot_y)

        hist.append(rpos)

        print(str(robot_position[0]) + " -> " + str(rpos))

        buf = disp.copy()
        buf = buf.transpose(Image.ROTATE_180)

        for i in range(len(hist)):
            r = hist[i]
            if i == len(hist) - 1:
                col = (0,255,0)
                mid = (0, 0, 0)
            else:
                 col = (200,255,200)
                 mid = (180,180,180)

            buf.putpixel((r[0], r[1] - 1), col)
            buf.putpixel((r[0] - 1, r[1]), col)
            buf.putpixel((r[0], r[1]), mid)
            buf.putpixel((r[0] + 1, r[1]), col)
            buf.putpixel((r[0], r[1] + 1), col)


        buf = buf.resize((map_width * scale, map_height * scale))
        buf.save("/tmp/pos.jpg")
        time.sleep(2)
        i = i + 1
        i = i % 10


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
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
