#! /usr/bin/env python


""" 

Load the gif file passed as first argument, change the resolution to 13x13, and save it to an output file.

The output file has the same name as the input file, but with the suffix "_fixed.gif" 

"""

import sys
from PIL import Image



def main():
    if len(sys.argv) < 2:
        print("Usage: python fix_resolution.py <input_file>")
        return
    input_file = sys.argv[1]
    output_file = input_file.replace(".gif", "_fixed.gif")
    # load the image
    im = Image.open(input_file)
    # iterate over the frames of the gif
    frames = []
    for frame in range(0, im.n_frames):
        im.seek(frame)
        # resize the image
        im_resized = im.resize((13, 13))
        # save the resized image
        frames.append(im_resized)
    # save the resized frames as a gif
    frames[0].save(output_file, save_all=True, append_images=frames[1:], duration=100, loop=1)



if __name__ == '__main__':
    main()



