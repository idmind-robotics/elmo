#! /usr/bin/env python


"""


This module loads an mp4 video file and crops it to a centered square. It saves the cropped video to a new file.

Aditionally, the module scales the video by a certain factor before cropping it.

Aditionally, the module can be used to remove a watermark, by changing the most uncommon color to black.

"""


import cv2
import numpy as np
import os
import sys



WIDTH = 800
HEIGHT = 480
SCALING_FACTOR = 1.0
FRAMERATE = 30


def get_framerate(name):
    cap = cv2.VideoCapture(name)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("%s fps: %d" % (name, fps))
    cap.release()
    return fps


def process_video():
    print("analysing framerate...")
    fps = get_framerate(IN_FILE)
    if fps != FRAMERATE:
        print("changing framerate to %d..." % FRAMERATE)
        os.system("ffmpeg -loglevel error -i %s -r %d -y %s" % (IN_FILE, FRAMERATE, "tmp.mp4"))
        cap = cv2.VideoCapture("tmp.mp4")
    else:
        cap = cv2.VideoCapture(IN_FILE)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(OUT_FILE, fourcc, FRAMERATE, (WIDTH, HEIGHT))    
    while cap.isOpened():
        ret, frame = cap.read()
        n_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        if ret:
            print("Frame: %d/%d" % (cap.get(cv2.CAP_PROP_POS_FRAMES), n_frames))
            # scale frame
            frame = cv2.resize(frame, (0, 0), fx=SCALING_FACTOR, fy=SCALING_FACTOR)
            # get frame height and width
            frame_height = frame.shape[0]
            frame_width = frame.shape[1]
            if frame_height > HEIGHT or frame_width > WIDTH:
                start_crop_width_idx = int((frame_width - WIDTH) / 2)
                end_crop_width_idx = int((frame_width + WIDTH) / 2)
                start_crop_height_idx = int((frame_height - HEIGHT) / 2)
                end_crop_height_idx = int((frame_height + HEIGHT) / 2) 
                frame = frame[start_crop_height_idx:end_crop_height_idx, start_crop_width_idx:end_crop_width_idx]
            out.write(frame)
        else:
            break    
    cap.release()
    out.release()    
    cv2.destroyAllWindows()
    print("cleaning up...")
    os.system("rm tmp.mp4")
    print("output written to %s" % OUT_FILE)


def main():
    process_video()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: %s <input_file>" % sys.argv[0])
        sys.exit(1)
    IN_FILE = sys.argv[1]
    OUT_FILE = IN_FILE[:-4] + "_fixed.mp4"
    main()
