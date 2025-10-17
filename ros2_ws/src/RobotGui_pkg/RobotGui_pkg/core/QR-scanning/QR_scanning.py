import cv2
from pyzbar.pyzbar import decode
import re

def QR_reader(cap):
    while True:
        ret, frame = cap.read()
        #Checking for capturing
        if not ret:
            print('Error: FAILED TO CAPTURE IMAGE')
            break

        #Decode QR codes
        decoded_objects = decode(frame)

        for obj in decoded_objects:
            #Get QR data
            qr_data = obj.data.decode('utf-8')
            print(f"\nQR Code Content: '{qr_data}'")

            '''
            Validate the formate
            result_type, x, y = Validate function
            '''

            #Get bounding box coordinates (for framing)
            rect_x,rect_y,rect_w,rect_h = obj.rect
            
            