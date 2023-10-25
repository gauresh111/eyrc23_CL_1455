#!/usr/bin/env python3
import argparse
import os
import time
import random
import requests
import ctypes
import platform
from PIL import Image
import requests
from io import BytesIO
import cv2
from imageio import imread
from PIL import Image
import cv2
import numpy as np
import requests
from matplotlib import pyplot as plt

def get_wallpaper():
    #Random number
    num = random.randint(1,99)
    payload = {'Authorization': 'UkLhZJEmfFw4XNquzrPwkCNJnq7s3BgKuXlYhuTqWYOhN68SHzESXKSs'}
    #Search query
    query = 'nature%20wallpaper%204k'
    #URL for PEXELS
    url = 'https://api.pexels.com/v1/search?per_page=1&page=' + str(num) + '&query=' + query
    res= requests.get(url, headers=payload)
    if res.status_code == 200:
        img_url = res.json().get('photos')[0]['src']['original']
        #Make request to get the image
        print(img_url)
        img=Image.open(requests.get(img_url, stream=True).raw)
        
        img.save("/home/gauresh/Desktop/wallpaper22/wallpaper.jpeg")
        #image =resize((420,250))
        
        
        # img = requests.get(img_url)
        #Write and save the imgae locally with name temp.jpg
    else:
        print('error in making http request')

def main():
    get_wallpaper()
 
if __name__ == "__main__":
    main()