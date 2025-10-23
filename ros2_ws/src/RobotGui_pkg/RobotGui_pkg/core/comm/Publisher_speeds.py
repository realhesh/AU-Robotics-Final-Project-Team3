import paho.mqtt.client as mqtt 
import time 

broker = "10.96.185.112"  ## use ip address of wifi or use the ip address of esp 
port = 1883 
Topic = "AUR/controls"

## we need the node "controller node" 

Current_Motor_Right_Speed = 0.0
Current_Motor_left_Speed = 0.0

def Publisher_Motor_Speeds(client):
    Right_Speed_Motor = Current_Motor_Right_Speed
    Left_Speed_Motor = Current_Motor_left_Speed
    message = f"Right_Speed_Motor:{Right_Speed_Motor} , Left_Speed_Motor:{Left_Speed_Motor}"
    print(f"Speeds Of Motors = {message}")
    client.publish(Topic,message)  ## Topic First 

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
