#! /usr/bin/env python
# -*- encoding: UTF-* -*-

import qi
import argparse
import sys

def main(session):
    navigation_service = session.service("ALNavigation")

    navigation_service.loadExploration("/home/nao/museum1.explo")

    navigation_service.startLocalization()
    guess = [0.,0.]
    navigation_service.relocalizeInMap(guess)

if __name__ == "__main__"
    parser = argpase.ArgumentParser()
    parser.add_argument("--ip", type=str, default="pepper.local",
                        help="Robot IP Address")
    parser.add_argument("--port", type=str, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi")
        sys.exit(1)
    main(session)
