#! /usr/bin/env python


import speech_recognition as sr


def main():
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    while True:
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)
            print(recognizer.energy_threshold)


if __name__ == "__main__":
    main()