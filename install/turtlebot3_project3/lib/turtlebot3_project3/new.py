#!/usr/bin/env python3

import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import rclpy
from rclpy.node import Node
from heapq import heappush, heappop
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

class Astar(Node):
    def __init__(self):
        super().__init__('Astar')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 100)
        # self.turtlebot3_model = self.get_parameter('model').value if self.get_parameter('model') else 'waffle'

        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

        self.vel_pub.publish(self.msg)

        self.thresholdDistance = 0.01
        self.nd = int(1 / self.thresholdDistance)
        self.sizex = 10 * self.nd
        self.thresholdAngle = 10
        self.na = int(360 / self.thresholdAngle)

        self.tot = 0.18

        self.actions = [
            [2, 0], [0, 2], [2, 2],
            [0, 3], [3, 0], [3, 3],
            [2, 3], [3, 2]
        ]

        self.start = None
        self.goal = None
        self.parent = {}

        self.start_algorithm()

    def start_algorithm(self):
        pass  # Your algorithm implementation goes here

    def ros_inputs(self, xi, yi, Thetai, UL, UR):
        r = 0.033 #in metres
        L = 0.287 #in metres
        dt = 10
        UL = UL*2*math.pi/60
        UR = UR*2*math.pi/60
        thetan = 3.14 * Thetai / 180
        theta_dot = (r / L) * (UR - UL) 
        change_theta = theta_dot + thetan
        x_dot = (r / 2) * (UL + UR) * math.cos(change_theta) 
        y_dot = (r / 2) * (UL + UR) * math.sin(change_theta) 
        vel_mag = math.sqrt(x_dot** 2 + y_dot** 2)
        F_theta = (180*change_theta/ 3.14)   
        return vel_mag, theta_dot, F_theta

    def normalize(self, Node):
        x,y,t = Node[0],Node[1],Node[2]
        x = round(x/self.thresholdDistance)* self.thresholdDistance
        y = round(y/self.thresholdDistance)* self.thresholdDistance
        t = round(t/self.thresholdAngle) * self.thresholdAngle
        x=round(x,4)
        y=round(y,4)
        n=t//360
        t=t-(360*n)
        t=(t/self.thresholdAngle)
        return [x,y,int(t)]

    def distance(self, start_coordinate, goal_coordinate):
        sx,sy = start_coordinate[0],start_coordinate[1]
        gx,gy = goal_coordinate[0],goal_coordinate[1]
        return math.sqrt((gx-sx)**2 + (gy-sy)**2)#instead of using scipy.spatial importing distance

    def listToString(self, s):  
            str1 = ""  
            for ele in s:  
                str1 += str(ele)  
            return str1  

    def boundary_check(self, i, j):
        if (i<self.tot or j>=6-self.tot or j<self.tot or i>=6-self.tot):
            return 0
        else:
            
            return 1


    def obs_map(self, x, y):
        circle1 = ((np.square(x-4.2))+ (np.square(y-1.2)) <=np.square(0.6+self.tot))
        # circle2 = ((np.square(x-3))+ (np.square(y-2)) <=np.square(1+self.tot))
        # circle3 = ((np.square(x-7))+ (np.square(y-2)) <=np.square(1+self.tot))
        # circle4 = ((np.square(x-7))+ (np.square(y-8)) <=np.square(1+self.tot))
        square1=(x>=1.5-self.tot) and (x<=1.75+self.tot) and (y>=1-self.tot) and (y <= 2)
        square2=(x>=2.5-self.tot) and (x<=2.75+self.tot) and (y>=0) and (y <= 1+self.tot)
        # square3=(x>=2.25-self.tot) and (x<=3.75+self.tot) and (y>=7.25-self.tot) and (y <= 8.75+self.tot)
        boundary=(x<=self.tot) or (x>=6-self.tot) or (y <= self.tot) or (y>=2-self.tot)
        if circle1 or square1 or square2 or boundary:
            obj_val =0
        else:
            obj_val = 1
    
        return obj_val


    def plot_curve(self, xi, yi, Thetai, UL, UR):
        t = 0
        r = 0.033
        L = 0.287
        dt = 1
        Xn=xi
        Yn=yi
        Thetan = math.radians(Thetai)
        while t<10:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += (0.5*r) * (UL + UR) * math.cos(Thetan) * dt
            Yn += (0.5*r) * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            plt.plot([Xs, Xn], [Ys, Yn], color="red")
        Thetan = math.degrees(Thetan)
        return [Xn, Yn, Thetan]


    def move_bot(self, xi, yi, Thetai, UL, UR, s, n):
        t = 0
        r = 0.033
        L = 0.287
        dt = 1
        Xn=xi
        Yn=yi
        length=0
        Thetan = math.radians(Thetai*self.thresholdAngle)
        while t<10:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += (0.5*r) * (UL + UR) * math.cos(Thetan) * dt
            Yn += (0.5*r) * (UL + UR) * math.sin(Thetan) * dt
            length+=self.distance([Xs,Ys],[Xn,Yn])
            status=self.boundary_check(Xn,Yn)
            flag=self.obs_map(Xn,Yn)
            if (status!=1) or (flag != 1):
                return None
            Thetan += (r / L) * (UR - UL) * dt
            s.append((Xs,Ys))
            n.append((Xn,Yn))
        Thetan = math.degrees(Thetan)
        # print(Xn, Yn, Thetan,length)
        return [Xn, Yn, Thetan,length]

    def convergence(self, a, goal_node):
        if((np.square(a[0]-goal_node[0]))+ (np.square(a[1]-goal_node[1])) <=np.square(0.15)) :
                        return 0
        else:
            return 1

    def run(self):
        ###############  TAKING INPUTS ################################
        # clearance of the robot
        print("####################################################")
        print("For Manual input enter 1 else enter 2")
        print("####################################################")
        inp=int(input())
        m=False
        if inp==1:
            m=True
        

        ########  pre init #####################
        if(not m):
                tot=0.18
                rpm1=2
                rpm2=3
                x_start = float(0.5)
                y_start = float(1)
                theta_start = int(0)
                x_goal=float(5.75)
                y_goal=float(1)
                theta_goal = int(0)
                actions=[[rpm1,0],[0,rpm1],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
                start=self.normalize([x_start,y_start,theta_start])
                norm_current_node=start
                goal=self.normalize([x_goal,y_goal,theta_goal])
                goal_node=[goal[0],goal[1],goal[2]]
        # print(move_bot(norm_current_node[0], norm_current_node[1], norm_current_node[2], actions[0][0], actions[0][1]))

        #################### user init ################################
        #Wheel RPM
        if(m):  
                # clearance of the robot 
                self.tot=float(input("Enter the clearance")) 
                
                rpm1=int(input("Please enter the rpm1:"))
                rpm2=int(input("Please enter the rpm2:"))
                actions=[[rpm1,0],[0,rpm1],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
                
                #Getting the start nodes from the user
                x_start = float(input("Please enter start point x coordinate:"))
             
                y_start = float(input("Please enter start point y coordinate:"))
           
                theta_start = int(input("Please enter start orientation in degrees:"))
                start_obs = self.obs_map(x_start,y_start)
                start_boundary = self.boundary_check(x_start,y_start)
                
                while(start_obs!=1 or start_boundary!=1):
                    print("Incorrect start point! Please enter a valid start point:")
                    x_start = float(input("Please enter start point x coordinate:"))
               
                    y_start = float(input("Please enter start point y coordinate:"))
                 
                    theta_start = int(input("Please enter start orientation in degrees:"))
                    start_obs = self.obs_map(x_start,y_start)
                    start_boundary = self.boundary_check(x_start,y_start)
                
                start=self.normalize([x_start,y_start,theta_start])
                # start_node=[int(start[0]),int(start[1]),start[2]]
                norm_current_node=start
                #Geting the goal nodes from the user
                x_goal=float(input("Please enter goal point x coordinate:"))
         
                y_goal=float(input("Please enter goal point y coordinate:"))
         
                theta_goal=0
                goal_obs=self.obs_map(x_goal,y_goal)
                goal_boundary=self.boundary_check(x_goal,y_goal)
                
                
                while(goal_obs!=1 or goal_boundary!=1):
                    print("Incorrect goal point! Please enter a valid goal point:")
                    x_goal=float(input("Please enter another goal point x coordinate:"))
               
                    y_goal=float(input("Please enter another goal point y coordinate:"))
                
                    theta_goal=0
                    goal_obs=self.obs_map(x_goal,y_goal)
                    goal_boundary=self.boundary_check(x_goal,y_goal)
                goal=self.normalize([x_goal,y_goal,theta_goal])
                goal_node=[goal[0],goal[1],goal[2]]
        ###################   INTIALISING COST,VISITED,PARENT AND QUEUE  ###############
        #Initializing cost as infinity
        cost_array=np.array(np.ones((self.sizex,self.sizex,self.na)) * np.inf)
        #Initializing visited nodes as empty array
        visited=np.array(np.zeros((self.sizex,self.sizex,self.na)))
        #Heuristic distance-Eucledian

        #Initializing total cost with cost and distance
        # totalcost=np.array(np.ones((sizex,sizex,na) * np.inf))
        totalcost=np.array(np.ones((self.sizex,self.sizex,self.na)) * np.inf)

        parent={}
        Q=[]
        # append start point and initialize it's cost to zero
        heappush(Q,(0,start))
        cost_array[int(self.nd*start[0])][int(self.nd*start[1])][start[2]]=0
        totalcost[int(self.nd*start[0])][int(self.nd*start[1])][start[2]]=0
        explored=[]
        ###################### A-STAR##################################
        start_time=time.time()    
        breakflag=0
        s=[]
        n=[]
        ps=[]
        pn=[]

        while self.convergence(norm_current_node,goal_node):
            if breakflag==1:
                break
            norm_current_node=heappop(Q)[1]
            if self.convergence(norm_current_node,goal_node)==0:
                goalfound=[norm_current_node,action]
                print("found")
                print(norm_current_node)
                break
            for action in actions:
                curr_node=self.move_bot(norm_current_node[0], norm_current_node[1], norm_current_node[2], action[0], action[1],s,n)
                if(curr_node==None):
                    continue
                L=curr_node[3]
                curr_node=curr_node[0:3]
                norm_curr_node=self.normalize(curr_node)
                if self.convergence(norm_curr_node,goal_node)==0:
                    print("found")
                    parent[self.listToString(norm_curr_node)]=[norm_current_node,action]
                    goalfound=[norm_curr_node,action]
                    print(norm_curr_node)
                    breakflag=1
                    break
                status=self.boundary_check(norm_curr_node[0],norm_curr_node[1])
                flag=self.obs_map(norm_curr_node[0],norm_curr_node[1])
                if (status and flag == 1):
                    if visited[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]==0:
                        visited[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=1
                        explored.append([norm_current_node,action,norm_curr_node])
                        parent[self.listToString(norm_curr_node)]=[norm_current_node,action]
                        cost_array[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=L+cost_array[int(self.nd*norm_current_node[0]),int(self.nd*norm_current_node[1]),norm_current_node[2]]
                        totalcost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=cost_array[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]] + self.distance(norm_curr_node, goal_node)
                        heappush(Q,( totalcost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]] ,norm_curr_node ))
                    else:
                        if totalcost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]>(totalcost[int(self.nd*norm_current_node[0]),int(self.nd*norm_current_node[1]),norm_current_node[2]]+sum(action)):
                            totalcost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=(totalcost[int(self.nd*norm_current_node[0]),int(self.nd*norm_current_node[1]),norm_current_node[2]]+sum(action))
                            explored.append([norm_current_node,action,norm_curr_node])
                            parent[self.listToString(norm_curr_node)]=[norm_current_node,action]


        ################## BACKTRACK ##########################
        path=[]
        def path_find(goal,start):
            path.append(goal)
            GN=parent[self.listToString(goal[0])]
            path.append(GN)
            while (GN[0]!=start):
                GN=parent[self.listToString(GN[0])]
                path.append(GN)
            return path


        path=path_find(goalfound,start)
        path.reverse()
        print("Time taken")
        print(time.time()-start_time)  

        sx=x_start
        sy=y_start
        sz=theta_start
        fig, ax = plt.subplots()
        ax.set(xlim=(0, 6), ylim=(0, 2))

        plt.plot(norm_curr_node[0], norm_curr_node[1], color='green', marker='o', linestyle='dashed', linewidth=1,markersize=4)
        plt.plot(start[0], start[1], color='yellow', marker='o', linestyle='dashed', linewidth=1,markersize=4)
        
        c1 = plt.Circle((4.2, 1.2), 0.6, fill=None)
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((1.5, 1), 0.25, 1, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((2.5, 0), 0.25, 1, fill=None, alpha=1))
        # currentAxis.add_patch(Rectangle((2.25, 7.25), 1.5, 1.5, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((0, 0), 6, 2, fill=None, alpha=1))
        
        ax.add_artist(c1)
        ax.set_aspect('equal')
        plt.grid()
        for action in path:
            x1= self.plot_curve(sx,sy,sz, action[1][0],action[1][1])
            sx=x1[0]
            sy=x1[1]
            sz=x1[2]
        plt.show()
        plt.pause(1)
        plt.close()
        ros=[]
        for i in range (len(path)):
            ros.append(path[i][1])
        print(ros)
        c =0
        progress=0
        for action in ros:
            print("action", action)
            print("Progress: ", int((progress/len(ros))*100), "%")
            progress+=1
            while rclpy.ok():
                if c== 101:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.vel_pub.publish(self.msg)
                    break
                else:
                    vel , th, F_theta = self.ros_inputs(0.5,1,0,action[0],action[1])
                    self.msg.linear.x = vel*10
                    self.msg.angular.z =  th*10
                    self.vel_pub.publish(self.msg)
                    c=c+1
                    time.sleep(0.1)
            c=0
        print("REACHED GOAL !")
        time.sleep(2)

        exit


    def check(self):
         print("Inside check function now!")

def main(args=None):
    rclpy.init(args=args)
    astar = Astar()

    astar.check()
    astar.run()

    rclpy.spin(astar)
    astar.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
