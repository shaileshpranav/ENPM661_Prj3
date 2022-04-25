import os
import cv2
import shutil
from glob import glob
# from tqdm import tqdm
import csv
import numpy as np

def string_to_int(input_str):
    return np.array([int(e) for e in input_str.split(',')])

def foldercheck(path):
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def deleteFolder(path):
    if os.path.exists(path):
        shutil.rmtree(path)


def remove_file(file):
    try:
        os.remove(file)
    except OSError as e:
        print("Error: %s : %s" % (file, e.strerror))

def createMovie(path,video_name="simulation_video", fps = 10):

    images = sorted(glob(path+"/*.png"))
    frame = cv2.imread(os.path.join(images[0]))
    height, width, channels = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(video_name+".mp4", fourcc, fps , (width,height))
    imgs = []
    for i in range(len(images)):
        image = cv2.imread(images[i])
        if i==0:
            h, w, _ = image.shape
        video.write(image)
        # imgs.append(imageio.imread(images[i]))
    # for i in range(20):
    #     imgs.append(imageio.imread(images[len(images)-1]))
    # imageio.mimsave(video_name+'gif', imgs, fps=fps )
    cv2.destroyAllWindows()
    video.release()


def deg2rad(rot): 
    return 3.14*rot/180

def rad2deg(theta_new):
    theta_new = 180*theta_new/3.14    
    if (theta_new >= 360): theta_new -= 360
    if (theta_new <= -360): theta_new += 360
    # theta_new = theta_new % 360 if theta_new > 0 else (theta_new + 360) % 360
    return theta_new

def half_round(x):
    x = round(2*x)/2    
    if x == 10 : x-=0.5
    return x


def gazebo2map(x, rev=False):
    if rev:
        return (x-5.0)/1.0
    return 1.0*x +5.0  


def read_file(file_name):

    rows= []
    with open(file_name, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            rows.append(row)

    return rows
