#!/usr/bin/env python
import os
import numpy as np
import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--camera", type=str, default="custombirdview", help="Which camera(s) to use in the env")
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--width", type=int, default=640)
args = parser.parse_args()

fourcc = cv2.VideoWriter_fourcc(*'DIVX')
fps = 50
img_w, img_h = args.width, args.height
img_size = (img_w, img_h)
out = cv2.VideoWriter("./images/" + args.camera +  "_np.mp4", fourcc, fps, img_size)

img_array = []
for frame_np in sorted(os.listdir("./images/" + args.camera), key=lambda num: int(num[:-4])):
    with open("./images/"+ args.camera + "/" + frame_np, 'rb') as f:
        frame_rgb = np.load(f)
        out.write(cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR))
        f.close
out.release()