import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from queue import PriorityQueue
import math
import time

boundry = []    
Pth = {}       
UncheckedList = PriorityQueue()     # to store unvisited nodes
backtrack_ = []            
CloseList = []
CheckedList = np.zeros((250,600),dtype='float64')     # to store the visited nodes
r_radius = 22
w_radius = 3.3
w_length = 28.7

startTime = time.time()

#Map
def canvas_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            space[l][m] = [224,224,224]
            if ((200-l) - 5 < 0) or ((m) - 5 < 0) or ((200-l) -195 > 0) or ((m) - 595 > 0): #boundary
                space[l][m] = [102,255,178]
                
            if (m > ((150-Obstacle_Clearance)-r_radius)) and (m < (175+Obstacle_Clearance+r_radius)) and (200-l < 200) and (200-l > ((100-Obstacle_Clearance)-r_radius)):   #rectangle1 with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (m > (150-Obstacle_Clearance)) and (m < (175+Obstacle_Clearance)) and (200-l < 200) and (200-l > (100-Obstacle_Clearance)):   #rectangle1 with Obstacle Clearance
                space[l][m] = [102,255,178]
                
            if (m > 150) and (m < 175) and (200-l < 200) and (200-l > 100):   #rectangle1
                space[l][m] = [255,204,153]
                
            if (m > ((250-Obstacle_Clearance)-r_radius)) and (m < (275+Obstacle_Clearance+r_radius)) and (200-l >0) and (200-l < (100+Obstacle_Clearance+r_radius)):  #rectangle2 with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (m > (250-Obstacle_Clearance)) and (m < (275+Obstacle_Clearance)) and (200-l >0) and (200-l < (100+Obstacle_Clearance)):  #rectangle2 with Obstacle clearance
                space[l][m] = [102,255,178]

            if (m > 250) and (m < 275) and (200-l >0) and (200-l < 100):  #rectangle2
                space[l][m] = [255,204,153]
            
            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow((60+Obstacle_Clearance+r_radius),2)) < 0:    #Circle with Obstacle clearance and Robot Radius
                space[l][m] = [192,192,192]

            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow((60+Obstacle_Clearance),2)) < 0: #Circle with Obstacle Clearance
                space[l][m] = [102,255,178]
                
            if (math.pow((m-420),2) + math.pow(((200-l)-120),2) - math.pow(60,2)) < 0: #Circle
                space[l][m] = [255,204,153]                
    return 


# Obstacle coordinates -> list
def boundry_creation(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if (space[l][m][0] != 224) and (space[l][m][2] > 20 or space[l][m][1] > 20 or space[l][m][0] > 20):
                boundry.append((m,200-l))
    return boundry


# Start node - INPUT
def User_Inputs_Start(Obs_Coords):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        z = int(input("Enter the orientation of robot at start pt.(in degrees): "))
        
        if((x>=0) and (x<=600)) and (y>=0) and (y<=200):
            if (x,y) not in Obs_Coords :
                start_node = (x,y,z)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")
# Goal node - INPUT
def User_Inputs_Goal(Obs_Coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        
     
        if((x>=0) and (x<=600)) and (y>=0) and (y<=200):
            if (x,y) not in Obs_Coords :
                goal_node=(x,y)
                break
            else:
                print("The Entered Goal Node is in obstacle space")
    return goal_node
# RPM - INPUT
def User_Input_rpm():
    rpm1 = int(input("Enter the first RPM: "))
    rpm2 = int(input("Enter the second RPM: "))
    return (rpm1,rpm2)

def angle_conversion(theta):
    if theta > 360:
        theta = theta % 360
        return theta
    elif theta < -360:
        theta = (-theta % 360)*(-1) 
        return theta
    else:
        return theta
    
def a_star_function(pos,ul,ur):
    old_x = round(pos[0])
    old_y = round(pos[1])
    theta = round(pos[2])
    dt=0.1
    theta_n = (np.pi * (theta/180))
    Cost = 0
    Xn = old_x
    Yn = old_y
    
    Xn += 0.5*w_radius*(ul+ur)*np.cos(theta_n)*dt
    Yn += 0.5*w_radius*(ul+ur)*np.sin(theta_n)*dt
        
    theta_n += (w_radius/w_length)*(ur-ul)
    theta_n = (180*(theta_n))/np.pi
    theta_n = angle_conversion(theta_n)
    if (0<=round(Xn)<600) and (0<=round(Yn)<200):
        if (CheckedList[int(round(Yn))][int(round(Xn))] != 1) and ((round(Xn),round(Yn)) not in Obs_Coords):
            Cost = Cost+math.sqrt(math.pow((0.5*w_radius * (ul + ur) * np.cos(theta_n) * dt),2)+math.pow((0.5*w_radius * (ul + ur) * np.sin(theta_n) * dt),2))
            CloseList.append((round(Xn),round(Yn),round(theta_n))) 
            Eucledian_dist = np.sqrt(((goal_pt[0] - Xn)*2)+((goal_pt[1] - Yn)*2))
            TotalCost = Cost + Eucledian_dist
            for m in range(UncheckedList.qsize()):
                if UncheckedList.queue[m][3] == (round(Xn),round(Yn),round(theta_n)):
                    if UncheckedList.queue[m][0] > TotalCost:
                        UncheckedList.queue[m] = (TotalCost,Eucledian_dist,Cost,(round(Xn),round(Yn),round(theta_n)))
                        Pth[(Xn,Yn,theta_n)] = (old_x,old_y,theta)
                        return
                    else:
                        return
            UncheckedList.put((TotalCost,Eucledian_dist,Cost,(round(Xn),round(Yn),round(theta_n))))
            Pth[(round(Xn),round(Yn),round(theta_n))] = (old_x,old_y,theta)
        
    
#left wheel rpm = 0 and right wheel rpm = rpm1
def zero_n_rpm1(a):
    pos = a[3]
    a_star_function(pos,0,vel_1)

#left wheel rpm = rpm1 and right wheel rpm = 0
def rpm1_n_zero(a):
    pos = a[3] 
    a_star_function(pos,vel_1,0)    
    
#left wheel rpm = rpm1 and right wheel rpm = rpm1
def rpm1_n_rpm1(a):
    pos = a[3]
    a_star_function(pos,vel_1,vel_1) 

# left wheel rpm = zero and right wheel rpm = rpm2
def zero_n_rpm2(a):
    pos = a[3]  
    a_star_function(pos,0,vel_2)
# left wheel rpm = rpm2 and right wheel rpm = zero
def rpm2_n_zero(a):
    pos = a[3]  
    a_star_function(pos,vel_2,0)

#left wheel rpm = rpm2 and right wheel rpm = rpm2
def rpm2_n_rpm2(a):
    pos = a[3] 
    a_star_function(pos,vel_2,vel_2)

#left wheel rpm = rpm1 and right wheel rpm = rpm2
def rpm1_n_rpm2(a):
    pos = a[3] 
    a_star_function(pos,vel_1,vel_2)
    
def rpm2_n_rpm1(a):
    pos = a[3] 
    a_star_function(pos,vel_2,vel_1)

# rpm to velocity
def rpm_to_velocity(rpm1,rpm2):
    v1 = (2*np.pi*w_radius*rpm1)/60
    v2 = (2*np.pi*w_radius*rpm2)/60
    return v1,v2


# Backtracking
def backtrack_(Pth,final_val,initial_pt):
    b_track = []
    z = list(Pth)[-1]
    K = Pth.get(z)
    b_track.append(z)
    b_track.append(K)
    print(b_track)
    while K != (initial_pt):  
        K = Pth.get(K)
        b_track.append(K)
    b_track.reverse()
    return (b_track)

         
space = np.ones((201,601,3),dtype='uint8') 


Obstacle_Clearance = int(input("Enter the Obstacle Clearance of the Robot: "))
canvas_space(space)          
Obs_Coords= boundry_creation(space)

initial_pt = User_Inputs_Start(Obs_Coords)  
goal_pt = User_Inputs_Goal(Obs_Coords)
rpm_1,rpm_2 = User_Input_rpm()
vel_1,vel_2 = rpm_to_velocity(rpm_1,rpm_2)

start = (0,0,0,initial_pt)
InitialEucledian_dist = np.sqrt(((goal_pt[0] - start[3][0])*2)+((goal_pt[1] - start[3][1])*2))  
InitialTotalCost = InitialEucledian_dist   
start = (InitialTotalCost,InitialEucledian_dist,0,initial_pt)

UncheckedList.put(start)
reached=0
while UncheckedList.qsize() != 0:
    a = UncheckedList.get()
    if CheckedList[int(round(a[3][1])), int(round(a[3][0]))] != 1:
        CheckedList[int(round(a[3][1])) , int(round(a[3][0]))] = 1
        if a[1] > 5:
            zero_n_rpm1(a)     
            rpm1_n_zero(a)
            rpm1_n_rpm1(a)
            zero_n_rpm2(a)
            rpm2_n_zero(a)
            rpm2_n_rpm2(a)
            rpm1_n_rpm2(a)
            rpm2_n_rpm1(a)
        else:
            print("Goal reached")
            reached=1
            break


if reached ==1:
    b = backtrack_(Pth,a, initial_pt)
    print(b)
    
    for i in CloseList:
        xi,yi,teta = i[0],i[1],i[2]
        j = Pth.get((xi,yi,teta))
        cv.line(space,(int(i[0]), 200-int(i[1])),(int(j[0]),200-int(j[1])),(0,0,255),1)
    
        ScaledUp = cv.resize(space,(1202,402))
        cv.imshow("Map",ScaledUp)
        if cv.waitKey(20) & 0xFF == ord('q'):
            break

    for j in b:
        space[200-int(j[1])][int(j[0])] = [0,255,0]
        plt.scatter(int(j[0]), int(j[1]),color="blue")
        cv.circle(space,(int(j[0]),200-int(j[1])), 1, (0,255,0), -1)
        SPACEscaledUp = cv.resize(space,(1202,402))
        cv.imshow("Space ", SPACEscaledUp )
        if cv.waitKey(50) & 0xFF == ord('q'):
            break


else:
    print("Error")

cv.destroyAllWindows()
endTime = time.time()
print("\nrun time = ", endTime - startTime, "seconds")