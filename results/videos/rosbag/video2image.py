#!/usr/bin/env python3 
# -*- coding:utf-8 -*-
import argparse
import cv2
import os
import os.path as osp



parser = argparse.ArgumentParser(description="Convert video to images.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument('input_video', help='input video file')
parser.add_argument('output_dir', help='output images directory')
args = parser.parse_args()    
if osp.exists(args.output_dir):
    print('Output directory already exists:', args.output_dir)
    sys.exit(1)

os.makedirs(args.output_dir)

vidcap = cv2.VideoCapture(args.input_video)

def getFrame(sec):
    vidcap.set(cv2.CAP_PROP_POS_MSEC,sec*1000)
    hasFrames,image = vidcap.read()
    if hasFrames:
        cv2.imwrite(args.output_dir + "/image%06i.jpg" % count, image)     # save frame as JPG file
        print("image%06i" % count + "saved.")
    return hasFrames


sec = 0
frameRate = 0.25 #//it will capture image in each 0.5 second
count=1
success = getFrame(sec)
while success:
    count = count + 1
    sec = sec + frameRate
    sec = round(sec, 2)
    success = getFrame(sec)
print("Success!")

