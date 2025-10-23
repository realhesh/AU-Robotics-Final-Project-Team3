import paho.mqtt.client as mqtt 
import time 

broker = "10.96.185.112"  ## use ip address of wifi or use the ip address of esp 
port = 1883 
Topic = "AUR/localization"

 

Current_Coordinate_X = 0.0
Current_Coordinate_Y = 0.0

def on_connect(client, userdata, flags, rc):

    if rc == 0 :
        print("Connected")
        client.subscribe(Topic)
        print(f"Connected to topic : {Topic} ")
    else :
        print("No Connectinon")  

def on_message(client,userdata,msg):
    message = msg.payload.decode()
    print(f"Coordinates : {message}")         

def Start_Subscriber():
    client = mqtt.Client()

    client.on_connect = on_connect
    client.on_message = on_message
    
    try :
        client.connect(broker,port,60)
    except :
        print("An Error Has Occured")
        return

    client.loop_forever()        
    
    
if __name__=="__main__":

    Start_Subscriber()
