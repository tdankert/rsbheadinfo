#! /usr/bin/env python
# -*- encoding: UTF-* -*-

import qi
import argparse
import sys

'''

'''

def main(session):
    awareness_service = session.service("ALBasicAwareness")
    autonomousLife_service = session.service("ALAutonomousLife")
    speakingMovement_service = session.service("ALSpeakingMovement")

    autonomousLife_service.setState('disabled')

    awareness_service.setTrackingMode('Head')
    awareness_service.setEnabled(False)

    speakingMovement_service.setEnabled(True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
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
