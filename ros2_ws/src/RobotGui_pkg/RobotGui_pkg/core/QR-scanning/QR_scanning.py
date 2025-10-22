import cv2
from pyzbar.pyzbar import decode
import re
import numpy as np

margin_cm = 1  
lower_color = np.array([0, 50, 50]);
upper_color = np.array([10, 255, 255]);
lower_color2 = np.array([170, 50, 50]);
upper_color2 = np.array([180, 255, 255]);

# Checking if the QR surrounded by green color (1 cm left and right)
def is_qr_surrounded_by_green(frame, qr_bbox, margin_cm=1, dpi=96):
    
    # Convert cm to pixels (1 inch = 2.54 cm)
    margin_px = int(margin_cm * dpi / 2.54)
    
    # Convert frame to RGB for processing
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    x, y, w, h = qr_bbox
    height, width = rgb_frame.shape[:2]
    
    # Define regions to check (1cm left and right of the QR code)
    left_region = rgb_frame[y:y+h, max(0, x - margin_px):x]
    right_region = rgb_frame[y:y+h, x+w:min(width, x+w + margin_px)]
    
    def is_region_green(region):
        if region.size == 0:
            return False
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(region, cv2.COLOR_RGB2HSV)
        
        # Create masks for the color
        mask1 = cv2.inRange(hsv, lower_color, upper_color)
        mask2 = cv2.inRange(hsv, lower_color2, upper_color2)
        color_mask = cv2.bitwise_or(mask1, mask2)
        
        # Calculate percentage of color pixels
        color_ratio = np.sum(color_mask > 0) / max(1, color_mask.size)
        
        # 60% of the pixels should be the specified color
        return color_ratio > 0.6  
    
    left_color = is_region_green(left_region)
    right_color = is_region_green(right_region)
    
    return left_color and right_color

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

            #Get bounding box coordinates (for framing)
            rect_x, rect_y, rect_w, rect_h = obj.rect
            qr_bbox = (rect_x, rect_y, rect_w, rect_h)

            # Check if QR code is surrounded by green
            is_green_background = is_qr_surrounded_by_green(frame, qr_bbox, margin_cm)
            
            #Get QR data
            qr_data = obj.data.decode('utf-8')
            print(f"\nQR Code Content: '{qr_data}'")

            #Validate the format
            result_type, x, y = validate_QR_format(qr_data)
            
            #Making a frame around the QR
            if result_type == 'valid_coordinates' and is_green_background:
                print(f"✅ VALID COORDINATES: x={x}, y={y} and green background")
                box_color = (0,255,0)
                status_text = f"VALID COORDINATES: x={x}, y={y} "

                # Draw box and display coordinates
                cv2.rectangle(frame, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), box_color, 3)
                cv2.putText(frame, status_text, (rect_x, rect_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                
            #Other content OR not green background
            else:
                print("📝 OTHER CONTENT OR NOT GREEN BACKGROUND")
                #Draw red box for other content
                cv2.rectangle(frame, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (0, 0, 255), 2)
                cv2.putText(frame, "OTHER CONTENT", (rect_x, rect_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        #Display the frame
        cv2.imshow('QR Code Format Validator', frame)

        #Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# To TEST
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    print("QR Code Format Validator")
    print("Looking for pattern: x=number&y=number (anywhere in text)")
    print("Also checking for 1cm green background around 4x4cm QR codes (centered in 6x6cm area)")
    print("Press 'q' to quit.")
    QR_reader(cap) 
   
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()