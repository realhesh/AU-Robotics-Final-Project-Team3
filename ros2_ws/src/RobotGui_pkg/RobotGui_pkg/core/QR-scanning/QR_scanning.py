import cv2
from pyzbar.pyzbar import decode
import re

def validate_QR_format(qr_data):
    #Pattern for x=#&y=# format (Anywhere in a string , Upper and Lower cases)
    pattern = r'[xX]=([\d.-]+)&[yY]=([\d.-]+)'

    match = re.search(pattern, qr_data.strip())
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return 'valid_coordinates', x, y
    else:
        return 'other_content', None, None
    
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

            
            #Validate the formate
            result_type, x, y = validate_QR_format()
            

            #Get bounding box coordinates (for framing)
            rect_x,rect_y,rect_w,rect_h = obj.rect

            #Making a frame around the QR
            if result_type == 'valid_coordinates':
                print(f"✅ VALID COORDINATES: x={x}, y={y}")
                #Draw green box and display cooredinates
                cv2.rectangle(frame,(rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (0, 255, 0), 3)
                cv2.putText(frame, f"Coordinates: x={x}, y={y}", (rect_x, rect_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            #Other content
            else:
                print("📝 OTHER CONTENT: No coordinate pattern found")
                #Draw red box for other content
                cv2.rectangle(frame, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (0, 0, 255), 2)
                cv2.putText(frame, "OTHER CONTENT", (rect_x, rect_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        #Display the frame
        cv2.imshow('QR Code Format Validator',frame)

        #Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

            