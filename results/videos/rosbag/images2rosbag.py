#!/usr/bin/env python3 
# -*- coding:utf-8 -*-

import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from PIL import ImageFile

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []

    if os.path.exists(dir):
        for path, _, files in os.walk(dir):
            files.sort(key=lambda x:x[:-4])
            print(files)
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    all.append( os.path.join( path, f ) )
    return all


def CreateMonoBag(imgs, bagname):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            fp = open( imgs[i], "r" )
            p = ImageFile.Parser()

            while 1:
                s = fp.read(1024)
                if not s:
                    break
                p.feed(s)

            im = p.close()

            Stamp = rospy.rostime.Time.from_sec(time.time())

            Img = Image()
            Img.header.stamp = Stamp
            Img.width = im.size[0]
            Img.height = im.size[1]

            # Img.width = 1280
            # Img.height = 720
            Img.encoding = "rgb8"
            Img.header.frame_id = "camera"
            Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            Img.data = Img_data

            bag.write('camera/image_raw', Img, Stamp)
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()
    CreateMonoBag(all_imgs, args[1])        


    # if len(left_imgs) > 0 and len(right_imgs) > 0:
    #     # create bagfile with stereo camera image pairs
    #     CreateStereoBag(left_imgs, right_imgs, args[1])
    # else:
    #     # create bagfile with mono camera image stream
    #     CreateMonoBag(all_imgs, args[1])        

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag( sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir bagfilename")