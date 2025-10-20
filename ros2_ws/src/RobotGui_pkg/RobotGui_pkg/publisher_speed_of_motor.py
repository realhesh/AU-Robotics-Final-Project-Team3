import paho.mqtt.client as mqtt 
import time 

broker = "192.168.1.18"  ## use ip address of wifi 
port = 1883 
Topic = "Speeds Of Motors"

Current_Motor_Right_Speed = 0.0
Current_Motor_left_Speed = 0.0

def Publisher_Motor_Speeds(client):
    
    client = mqtt.Client()
    
    Right_Speed_Motor = Current_Motor_Right_Speed
    Left_Speed_Motor = Current_Motor_left_Speed
    message = f"Right_Speed_Motor:{Right_Speed_Motor} , Left_Speed_Motor:{Left_Speed_Motor}"
    print(f"Speeds Of Motors = {message}")
    client.publish(message)
    

def Start_Publisher():
    
    client = mqtt.Client()
    try :
        client.connect(broker,port,60)
        connection_successful = True
    except Exception as e :
        connection_successful = False
    
    if connection_successful :
        while True :
          client.loop_start()
          Publisher_Motor_Speeds(client)
          time.sleep(0.1)
    else : 
        print("No Connection , An Error Has Occured")   

if __name__=="__main__":
    Start_Publisher()




















































