import socket
import time
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import cv2




position=ctrl.Antecedent(np.arange(580, 900, 1), 'position')

position['H']=fuzz.smf(position.universe, 750, 900)
position['M']=fuzz.gaussmf(position.universe, 750, 40)
position['L']=fuzz.zmf(position.universe, 600, 750)

output=ctrl.Consequent(np.arange(0, 1, 0.01), 'output')

output['H']=fuzz.smf(output.universe, 0.6, 1)
output['M']=fuzz.gaussmf(output.universe, 0.6, 0.2)
output['L']=fuzz.zmf(output.universe, 0.4, 0.6)

rule1=ctrl.Rule(position['H'], output['H'])
rule2=ctrl.Rule(position['M'], output['M'])
rule3=ctrl.Rule(position['L'], output['L'])

fuzzy_ctrl=ctrl.ControlSystem([rule1, rule2, rule3])

simulation=ctrl.ControlSystemSimulation(fuzzy_ctrl)

# Set up network socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('172.20.10.3', 5555)) #

net = cv2.dnn.readNet("dnn_model/yolov4-tiny.weights", "dnn_model/yolov4-tiny.cfg")
model = cv2.dnn_DetectionModel(net)
model.setInputParams(size = (320,320), scale = 1/255)
#load class names
classes =[]

with open("dnn_model/classes.txt", "r") as file_object:
    for class_name in file_object.readlines():
        class_name = class_name.strip()
        classes.append(class_name)
 
#initialize Camera

while True:

    cap = cv2.VideoCapture(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cv2.namedWindow('frame')
    need_ctrl=True
    # cv2.setMouseCallback('frame', click_event)
    while need_ctrl :
        #get Frame
        ret, frame = cap.read()
        #object detection
        (class_ids, scores, bboxes)= model.detect(frame)
        for class_id, score, bbox in zip(class_ids, scores,bboxes):
            if classes[class_id] == "bottle" or classes[class_id] == 'cup':
                (x,y,w,h) = bbox
                class_name = classes[class_id]
                cv2.putText(frame,classes[class_id],(x,y-10), cv2.FONT_HERSHEY_COMPLEX, 2, (200, 0, 50), 2)
                cv2.rectangle(frame, (x,y),(x+w, y+h), (200, 0, 50), 3)
                
                #get position
                position = x 
                if position < 420:
                    need_ctrl = False
                    print(class_name)
                    #stop motor
                    client_socket.sendall(b'stop\n')
                    time.sleep(1)
                    if class_name == 'bottle':
                        client_socket.sendall(b'bottle\n')
                    else:
                        client_socket.sendall(b'cup\n')
                    time.sleep(1)
                    client_socket.sendall(b'stop_code\n')
                    
                else:
                    #fuzzy control
                    simulation.input['position']=position
                    simulation.compute()
                    output = simulation.output['output']
                    #send output to motor
                    client_socket.sendall(str(output).encode() + b'\n')
                    time.sleep(0.1)

    
            
    
        
        cv2.imshow("frame",frame)
        key=cv2.waitKey(1)
        if key == ord('q'):
            break 
    # release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()
    
    while True:
        status = client_socket.recv(1024).decode().strip()
        if status == 'done':
            print('The Next Iteration is ready')
            break
    
# client_socket.sendall(b'done\n')


# Close network socket
client_socket.close()