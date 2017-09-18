#! /usr/bin/env python
# -*- encoding: UTF-* -*-

import qi
import argparse
import sys

def main(session):
    asr_service = session.service("ALSpeechRecognition")
    tts_service = session.service("ALTextToSpeech")
    asr_service.setVocabulary(["Ja","Nein","Schwert","Schild"], False)
    tts_service.setLanguage("German")

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
