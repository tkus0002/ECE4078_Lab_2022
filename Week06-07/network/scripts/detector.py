import os 
import time
from xml.etree.ElementTree import XML

import cmd_printer
import numpy as np
import torch
from args import args
from res18_skip import Resnet18Skip
import cv2

class Detector:
    def __init__(self, ckpt, use_gpu=False):
        #Loading the custom Yolov5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', ckpt, force_reload=True)

        if torch.cuda.torch.cuda.device_count() > 0 and use_gpu:
            self.use_gpu = True
            self.model = self.model.cuda()
        else:
            self.use_gpu = False
        self.load_weights(ckpt)
        #The model is then set to evaluation mode
        self.model = self.model.eval()

    def detect_single_image(self, np_img):
        #With Yolo no need for any conversions model will handle it
        tick = time.time()
        with torch.no_grad():
            pred = self.model.forward(np_img)
        
        dt = time.time() - tick
        print(f'Inference Time {dt:.2f}s, approx {1/dt:.2f}fps', end="\r")
        #Getting the coordinates for the bounding box produced by Yolo
        num_preds = len(pred.pandas().xyxy[0])
        #Making a result array to return the predictions
        pred_results = np.array[(num_preds,5)]
        #Going throuch prediction and gettring the bounding box and class prediction
        for i in range(num_preds):
            #Get the class
            predic_class = int(pred.pandas().xyxy[0]["class"][i])
            #Get the corners of the bounding box
            xl = pred.pandas().xyxy[0]['xmin'][i]
            xu = pred.pandas().xyxy[0]['xmax'][i]
            yl = pred.pandas().xyxy[0]['ymin'][i]
            yu = pred.pandas().xyxy[0]['ymax'][i]
            #Combining the outputs into a np array with class, xmin,xmax,ymin,ymax
            pred_results[i] = [predic_class,xl,xu,yl,yu]
        
        return pred.render()[0],pred_results


