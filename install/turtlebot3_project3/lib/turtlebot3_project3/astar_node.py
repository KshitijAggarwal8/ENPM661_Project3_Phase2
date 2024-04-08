#!/usr/bin/env python3

import numpy as np
import rclpy
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import math
import time
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.node import Node
from heapq import heappush, heappop


class A_star(Node):
    def __init__(self):
        super().__init__('A_star')

        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0

        # Initialising the velocity publisher to publish commands on /cmd_vel
        self.velocity_publish = self.create_publisher(Twist, 'cmd_vel', 100)
        self.velocity_publish.publish(self.msg)

        self.branch_length = 0.01
        self.nd = int(1 / self.branch_length)
        self.sizex = 10 * self.nd
        self.branch_arc = 10
        self.na = int(360 / self.branch_arc)

        self.clearance = 0.18

        # Initialising default actions
        self.actions = [
            [2, 0], [0, 2], [2, 2],
            [0, 1], [1, 0], [1, 1],
            [2, 1], [1, 2]
        ]

        self.start = None
        self.goal = None
        self.parent = {}

    def convert_to_string(self, s):  
            str1 = ""  
            for ele in s:  
                str1 += str(ele)  
            return str1  

    # checking if the given point is inside the arena boundary
    def check_boundary(self, i, j):
        if (i<self.clearance or j>=6-self.clearance or j<self.clearance or i>=6-self.clearance):
            return 0
        else:
            return 1

    # Calculating the differential drive values to feed the controller
    def diff_drive(self, xi, yi, Thetai, UL, UR):
        radius = 0.033 
        wheel_gap = 0.287 
        dt = 10
        UL = UL*2*math.pi/60
        UR = UR*2*math.pi/60
        thetan = 3.14 * Thetai / 180
        theta_dot = (radius / wheel_gap) * (UR - UL) 
        change_theta = theta_dot + thetan
        x_dot = (radius / 2) * (UL + UR) * math.cos(change_theta) 
        y_dot = (radius / 2) * (UL + UR) * math.sin(change_theta) 
        vel_mag = math.sqrt(x_dot** 2 + y_dot** 2)
        F_theta = (180*change_theta/ 3.14)   
        return vel_mag, theta_dot, F_theta

    def unit(self, Node):
        x,y,t = Node[0],Node[1],Node[2]
        x = round(x/self.branch_length)* self.branch_length
        y = round(y/self.branch_length)* self.branch_length
        t = round(t/self.branch_arc) * self.branch_arc
        x=round(x,4)
        y=round(y,4)
        n=t//360
        t=t-(360*n)
        t=(t/self.branch_arc)
        return [x,y,int(t)]

    # Defining the obstacle zone
    def occupied_region(self, x, y):
        circle1 = ((np.square(x-4.2))+ (np.square(y-1.2)) <=np.square(0.6+self.clearance))
        square1=(x>=1.5-self.clearance) and (x<=1.75+self.clearance) and (y>=1-self.clearance) and (y <= 2)
        square2=(x>=2.5-self.clearance) and (x<=2.75+self.clearance) and (y>=0) and (y <= 1+self.clearance)
        
        boundary=(x<=self.clearance) or (x>=6-self.clearance) or (y <= self.clearance) or (y>=2-self.clearance)
        if circle1 or square1 or square2 or boundary:
            flag =0
        else:
            flag = 1
    
        return flag
        
    # Calculates the distance between goal and current point
    def gap_between(self, start_coordinate, goal_coordinate):
        start_x,start_y = start_coordinate[0],start_coordinate[1]
        gx,gy = goal_coordinate[0],goal_coordinate[1]
        return math.sqrt((gx-start_x)**2 + (gy-start_y)**2)

    # Plots the optimal path and opens in a window
    def curve_plot(self, xi, yi, Thetai, UL, UR):
        t = 0
        radius = 0.033
        wheel_gap = 0.287
        dt = 1
        Xn=xi
        Yn=yi
        theta_unit = math.radians(Thetai)
        while t<10:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += (0.5*radius) * (UL + UR) * math.cos(theta_unit) * dt
            Yn += (0.5*radius) * (UL + UR) * math.sin(theta_unit) * dt
            theta_unit += (radius / wheel_gap) * (UR - UL) * dt
            plt.plot([Xs, Xn], [Ys, Yn], color="red")
        theta_unit = math.degrees(theta_unit)
        return [Xn, Yn, theta_unit]

    # Initialising and calculating the appropriate movement commands for the robot
    def bot_motion(self, xi, yi, Thetai, UL, UR, s, n):
        t = 0
        radius = 0.033
        wheel_gap = 0.287
        dt = 1
        Xn=xi
        Yn=yi
        length=0
        theta_unit = math.radians(Thetai*self.branch_arc)
        while t<10:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += (0.5*radius) * (UL + UR) * math.cos(theta_unit) * dt
            Yn += (0.5*radius) * (UL + UR) * math.sin(theta_unit) * dt
            length+=self.gap_between([Xs,Ys],[Xn,Yn])
            status=self.check_boundary(Xn,Yn)
            flag=self.occupied_region(Xn,Yn)
            if (status!=1) or (flag != 1):
                return None
            theta_unit += (radius / wheel_gap) * (UR - UL) * dt
            s.append((Xs,Ys))
            n.append((Xn,Yn))
        theta_unit = math.degrees(theta_unit)
    
        return [Xn, Yn, theta_unit,length]

    # Check wheter the goal is reached
    def status_goal(self, a, goal_node):
        if((np.square(a[0]-goal_node[0]))+ (np.square(a[1]-goal_node[1])) <=np.square(0.05)) :
                        return 0
        else:
            return 1

    # Main A* algorithm
    def algorithm(self):

        print(" ")
        print(" _________________________")
        print("|                         |")
        print("| A* Gazebo visualisation |")
        print("|_________________________|")
        print(" ")
        
        # clearance of the robot 
        self.clearance=float(input("Enter the clearance")) 
        
        rpm1=int(input("Enter RPM1:"))
        rpm2=int(input("Enter RPM2:"))
        actions=[[rpm1,0],[0,rpm1],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
        
        # Getting the start nodes from the user in metres
        x_start = float(input("Enter start point x coordinate (in metres):"))
        
        y_start = float(input("Enter start point y coordinate (in metres):"))
    
        theta_start = int(input("Enter start orientation in degrees:"))
        obstacle = self.occupied_region(x_start,y_start)
        boundary_st = self.check_boundary(x_start,y_start)
        
        # If the given point is inside the obstacle space, then re-enter
        while(obstacle!=1 or boundary_st!=1):
            print("Invalid start point! Enter a valid start point:")
            x_start = float(input("Enter start point x coordinate (in metres):"))
        
            y_start = float(input("Enter start point y coordinate (in metres):"))
            
            theta_start = int(input("Enter start orientation in degrees:"))
            obstacle = self.occupied_region(x_start,y_start)
            boundary_st = self.check_boundary(x_start,y_start)
        
        start=self.unit([x_start,y_start,theta_start])
      
        unit_node_current=start

        #Geting the goal nodes from the user in meteres
        x_goal=float(input("Enter goal point x coordinate (in metres):"))
    
        y_goal=float(input("Enter goal point y coordinate (in metres):"))
    
        theta_goal=0
        obstacle_goal=self.occupied_region(x_goal,y_goal)
        boundary_goal=self.check_boundary(x_goal,y_goal)
        
        while(obstacle_goal!=1 or boundary_goal!=1):
            print("Invalid goal point! Enter a valid goal point:")
            x_goal=float(input("Enter another goal point x coordinate (in metres):"))
        
            y_goal=float(input("Enter another goal point y coordinate (in metres):"))
        
            theta_goal=0
            obstacle_goal=self.occupied_region(x_goal,y_goal)
            boundary_goal=self.check_boundary(x_goal,y_goal)

        goal=self.unit([x_goal,y_goal,theta_goal])
        goal_node=[goal[0],goal[1],goal[2]]

       
        # Initializing cost as infinity
        cost_fx=np.array(np.ones((self.sizex,self.sizex,self.na)) * np.inf)
        # Initializing visited nodes 
        visited_fx=np.array(np.zeros((self.sizex,self.sizex,self.na)))
     
        # Initializing total cost
        total_cost=np.array(np.ones((self.sizex,self.sizex,self.na)) * np.inf)

        parent={}
        Q=[]
    
        heappush(Q,(0,start))

        cost_fx[int(self.nd*start[0])][int(self.nd*start[1])][start[2]]=0
        total_cost[int(self.nd*start[0])][int(self.nd*start[1])][start[2]]=0
        explored_fx=[]
       
       # Flag to calculate the total execution time
        start_time=time.time()    
        breakflag=0
        s=[]
        n=[]
        ps=[]
        pn=[]

        # While goal reached
        while self.status_goal(unit_node_current,goal_node):
            if breakflag==1:
                break

            # If goal reached, exit
            unit_node_current=heappop(Q)[1]
            if self.status_goal(unit_node_current,goal_node)==0:
                goalfound=[unit_node_current,action_set]
                print(" Goal found!")
                print(unit_node_current)
                break

            # Check for every action based on the current and the goal location
            for action_set in actions:
                
                curr_node=self.bot_motion(unit_node_current[0], unit_node_current[1], unit_node_current[2], action_set[0], action_set[1],s,n)
                if(curr_node==None):
                    continue
                L=curr_node[3]
                curr_node=curr_node[0:3]
                norm_curr_node=self.unit(curr_node)

                if self.status_goal(norm_curr_node,goal_node)==0:
                    print("Goal found!")
                    parent[self.convert_to_string(norm_curr_node)]=[unit_node_current,action_set]
                    goalfound=[norm_curr_node,action_set]
                    print(norm_curr_node)
                    breakflag=1
                    break

                status=self.check_boundary(norm_curr_node[0],norm_curr_node[1])
                flag=self.occupied_region(norm_curr_node[0],norm_curr_node[1])

                # if it is a safe point
                if (status and flag == 1):
                    if visited_fx[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]==0:
                        visited_fx[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=1
                        explored_fx.append([unit_node_current,action_set,norm_curr_node])
                        parent[self.convert_to_string(norm_curr_node)]=[unit_node_current,action_set]
                        cost_fx[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=L+cost_fx[int(self.nd*unit_node_current[0]),int(self.nd*unit_node_current[1]),unit_node_current[2]]
                        total_cost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=cost_fx[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]] + self.gap_between(norm_curr_node, goal_node)
                        heappush(Q,( total_cost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]] ,norm_curr_node ))
                    else:
                        if total_cost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]>(total_cost[int(self.nd*unit_node_current[0]),int(self.nd*unit_node_current[1]),unit_node_current[2]]+sum(action_set)):
                            total_cost[int(self.nd*norm_curr_node[0]),int(self.nd*norm_curr_node[1]),norm_curr_node[2]]=(total_cost[int(self.nd*unit_node_current[0]),int(self.nd*unit_node_current[1]),unit_node_current[2]]+sum(action_set))
                            explored_fx.append([unit_node_current,action_set,norm_curr_node])
                            parent[self.convert_to_string(norm_curr_node)]=[unit_node_current,action_set]


        # Backtrack and store the optimal path
        path=[]
        def find_path(goal,start):

            path.append(goal)
            goal_path_tracked=parent[self.convert_to_string(goal[0])]
            path.append(goal_path_tracked)

            while (goal_path_tracked[0]!=start):
                goal_path_tracked=parent[self.convert_to_string(goal_path_tracked[0])]
                path.append(goal_path_tracked)
            return path


        path=find_path(goalfound,start)
        path.reverse()  # Reverse path for ease of access

        print("Total time taken:")
        print(time.time()-start_time)  

        # Plotting the obstacle space and the optimal path onto display window
        start_x=x_start
        start_y=y_start
        sz=theta_start
        fig, ax = plt.subplots()
        ax.set(xlim=(0, 6), ylim=(0, 2))

        plt.plot(norm_curr_node[0], norm_curr_node[1], color='blue', marker='o', linestyle='dashed',markersize=5)
        plt.plot(start[0], start[1], color='yellow', marker='o', linestyle='dashed', markersize=4)
        
        # Defining the circle and rectangle obstacles 
        c1 = plt.Circle((4.2, 1.2), 0.6, fill=None)
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((1.5, 1), 0.25, 1, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((2.5, 0), 0.25, 1, fill=None, alpha=1))
        currentAxis.add_patch(Rectangle((0, 0), 6, 2, fill=None, alpha=1))
        
        ax.add_artist(c1)
        ax.set_aspect('equal')
        plt.grid()
        for action_set in path:
            x1= self.curve_plot(start_x,start_y,sz, action_set[1][0],action_set[1][1])
            start_x=x1[0]
            start_y=x1[1]
            sz=x1[2]
        plt.show()
        plt.pause(1)
        plt.close()


        # Initialise the ROS2 component
        to_publish=[]
        for i in range (len(path)):
            to_publish.append(path[i][1])
        print(to_publish)

        counter =0  # tracker for command duration
        progress=0  # tracker for %age completion
        for action_set in to_publish:
            print("action-", action_set)
            print("Progress: ", int((progress/len(to_publish))*100), "%")
            progress+=1
            while rclpy.ok():
                if counter== 101:
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                    self.velocity_publish.publish(self.msg)
                    break
                else:   # Publish the converted commands to the "/cmd_vel" topic
                    velocity , angle, F_theta = self.diff_drive(0.5,1,0,action_set[0],action_set[1])
                    self.msg.linear.x = velocity*10
                    self.msg.angular.z =  angle*10
                    self.velocity_publish.publish(self.msg)
                    counter=counter+1
                    time.sleep(0.1)
            counter=0
        print("REACHED GOAL !")
   
        time.sleep(2)

        sys.exit()

def main(args=None):
    rclpy.init(args=args)
    astar = A_star()

    astar.algorithm()

    rclpy.spin(astar)
    astar.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
