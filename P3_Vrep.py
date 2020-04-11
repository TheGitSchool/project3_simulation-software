
# In[1]:


import math
import numpy as np
import cv2 as cv
from scipy.spatial import distance
import vrep
import time


# In[2]:


#measurements in centimeters
r=3.8
l=23
dt=0.5
d=20


# In[3]:


def Goal(check_x,check_y):
    if (check_x-goal[0])**2+(check_y-goal[1])**2-49<=0:
        return 1
    else:
        return 0


# In[4]:
#obstacle map

def obstacle_map(x,y):
    circle1 = (x-500)**2+(y-500)**2-(100+d)**2<=0
    circle2 = (x-700)**2+(y-200)**2-(100+d)**2<=0
    circle3 = (x-300)**2+(y-800)**2-(100+d)**2<=0
    circle4 = (x-700)**2+(y-800)**2-(100+d)**2<=0
    
    square1 = y>=125-d and y<=175+d and x>=225-d and x<= 375+d
    square2 = y>=420-d and y<=500+d and x>=20-d and x<= 180+d
    square3 = y>=420-d and y<=500+d and x>=820-d and x<= 900+d
  
    
    boundary1 = x>=0 and x<=d 
    boundary2 = y>=0 and y<=d
    boundary3 = x>=1000-d and x<=1000 
    boundary4 = y>=1000-d and y<=1000

    if circle1 or circle2 or circle3 or circle4  or square1 or square2 or square3 or boundary1 or boundary2 or boundary3 or boundary4:
        obs_output = 0
    else:
        obs_output = 1

    return obs_output



# In[7]:
#boundary check value

def boundary_check(i,j):
    if (i<d or j>999-d or j<d or i>999-d):
        return 0
    else:
        return 1


# In[8]:

# Creating map list to store parent details
parent_list=[]

for j in range (1000):
    column=[]
    for i in range (1000):
        column.append(0)
    parent_list.append(column)


# In[9]:
#initializing start point
    
print("Taking the starting point of the TurtleBot as [50,50]")

start=[50,50]

x_start=start[0]
y_start=start[1]


# In[10]:


#Getting user inputs for goal points

x_goal=int(input("Enter  x-coordinate goal position"))
y_goal=int(input("Enter  y-coordinate goal position"))

goal_obs=obstacle_map(x_goal,y_goal)
goal_boundary=boundary_check(x_goal,y_goal)


while( ((goal_obs) and (goal_boundary)) !=1):
    print("Incorrect goal point. . Enter a valid goal point")
    x_goal=int(input("Enter another goal point x coordinate"))
    y_goal=int(input("Enter another goal point y coordinate"))
    goal_obs=obstacle_map(x_goal,y_goal)
    goal_boundary=boundary_check(x_goal,y_goal)

goal=[x_goal,y_goal]


# In[11]:
#the whole map array

cost=np.array(np.ones((1000,1000)) * np.inf)
euclidean_distance_arr=np.array(np.ones((1000,1000)) * np.inf)
total_cost_arr=np.array(np.ones((1000,1000)) * np.inf)
visited_arr=np.array(np.zeros((1000,1000)))


# In[12]:


# priority queue function
Q=[]
Q.append([x_start,y_start,0])
cost[x_start][y_start]=0
total_cost_arr[x_start][y_start]=0

def GetFirst(Q):
    index_min=0
    X_min = Q[0][0] 
    Y_min = Q[0][1]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        if total_cost_arr[x,y] < total_cost_arr[X_min,Y_min]:
            index_min = i
            X_min = x 
            Y_min= y
    current_node = Q[index_min]
    Q.remove(Q[index_min])
    return current_node


# In[13]:


## Setting two RPMs and calculating rad/sec

RPM1=60
RPM2=30

#Conversion to radians per sec from rpm
r1=(2*22*RPM1)/(60*7)   
r2=(2*22*RPM2)/(60*7)


# In[14]:


## Motion formulas for differential drive

def Differential_motion(r1,r2,theta):
    dtheta=(r*(r1-r2)*dt)/l + theta
    dx=(r*(r1+r2)*math.cos(dtheta)*dt)/2
    dy=(r*(r1+r2)*math.sin(dtheta)*dt)/2
    return dtheta,dx,dy


# In[15]:


# Movements of the Turtle Bot Robot

def straight(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'straight_0')
    return new_node

def straight_faster(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'straight_1')
    return new_node

def right(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'right_0')
    return new_node

def right1(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'right_1')
    return new_node

def right2(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'right_2')
    return new_node

def left(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'left_0')
    return new_node

def left1(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'left_1')
    return new_node

def left2(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)),'left_2')
    return new_node


# In[16]:


# Generating new nodes and appending to the queue list

visited_nodes=[]
current_node=[x_start,y_start,0]
while True: 
    if Goal(current_node[0],current_node[1])==1:
        goalbound=current_node
        break
    current_node=GetFirst(Q)
    straight_new=straight (current_node[0],current_node[1],current_node[2])
    status=boundary_check(straight_new[0],straight_new[1])
    flag=obstacle_map(straight_new[0],straight_new[1])
    if ( ((status) and (flag)) == 1):
        if visited_arr[straight_new[0],straight_new[1]]==0:
            visited_arr[straight_new[0],straight_new[1]]=1
            visited_nodes.append(straight_new)
            Q.append(straight_new)
            parent_list[straight_new[0]][straight_new[1]]=current_node
            
            cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_distance_arr[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
            total_cost_arr[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_distance_arr[straight_new[0],straight_new[1]]
        else:
            if cost[straight_new[0],straight_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_distance_arr[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
                parent_list[straight_new[0]][straight_new[1]]=current_node
                total_cost_arr[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_distance_arr[straight_new[0],straight_new[1]]
    
    
    straight_faster_new=straight_faster(current_node[0],current_node[1],current_node[2])
    status=boundary_check(straight_faster_new[0],straight_faster_new[1])
    flag=obstacle_map(straight_faster_new[0],straight_faster_new[1])
    
    if ( ((status) and (flag)) == 1):
        if visited_arr[straight_faster_new[0],straight_faster_new[1]]==0:
            visited_arr[straight_faster_new[0],straight_faster_new[1]]=1
            visited_nodes.append(straight_faster_new)
            Q.append(straight_faster_new)
            parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
            cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
            total_cost_arr[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]
        else:
            if cost[straight_faster_new[0],straight_faster_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
                parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
                total_cost_arr[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_distance_arr[straight_faster_new[0],straight_faster_new[1]]
    
    right_new=right(current_node[0],current_node[1],current_node[2])
    status=boundary_check(right_new[0],right_new[1])
    flag=obstacle_map(straight_new[0],straight_new[1])

    if ( ((status) and (flag)) == 1):
        if visited_arr[right_new[0],right_new[1]]==0:
            visited_arr[right_new[0],right_new[1]]=1
            visited_nodes.append(right_new)
            Q.append(right_new)         
            parent_list[right_new[0]][right_new[1]]=current_node
            cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_distance_arr[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
            total_cost_arr[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_distance_arr[right_new[0],right_new[1]]

        else:
            if cost[right_new[0],right_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[right_new[0]][right_new[1]]=current_node
                euclidean_distance_arr[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
                total_cost_arr[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_distance_arr[right_new[0],right_new[1]]
    
    
    
    left_new=left(current_node[0],current_node[1],current_node[2])
    status=boundary_check(left_new[0],left_new[1])
    flag=obstacle_map(left_new[0],left_new[1])
    if ( ((status) and (flag)) == 1):
        if visited_arr[left_new[0],left_new[1]]==0:
            visited_arr[left_new[0],left_new[1]]=1
            visited_nodes.append(left_new)
            Q.append(left_new)
            parent_list[left_new[0]][left_new[1]]=current_node
            euclidean_distance_arr[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
            cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
            total_cost_arr[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_distance_arr[left_new[0],left_new[1]]
        else:
            if cost[left_new[0],left_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[left_new[0]][left_new[1]]=current_node
                euclidean_distance_arr[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
                total_cost_arr[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_distance_arr[left_new[0],left_new[1]]
    
    
    right1_new=right1(current_node[0],current_node[1],current_node[2])
    status=boundary_check(right1_new[0],right1_new[1])
    flag=obstacle_map(right1_new[0],right1_new[1])

    if ( ((status) and (flag)) == 1):   
        if visited_arr[right1_new[0],right1_new[1]]==0:
            visited_arr[right1_new[0],right1_new[1]]=1
            visited_nodes.append(right1_new)
            Q.append(right1_new)
            parent_list[right1_new[0]][right1_new[1]]=current_node
            euclidean_distance_arr[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
            cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_distance_arr[right1_new[0],right1_new[1]]
        else:
            if cost[right1_new[0],right1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right1_new[0]][right1_new[1]]=current_node
                euclidean_distance_arr[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
                total_cost_arr[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_distance_arr[right1_new[0],right1_new[1]]
    
    
    right2_new=right2(current_node[0],current_node[1],current_node[2])
    status=boundary_check(right2_new[0],right2_new[1])
    flag=obstacle_map(right2_new[0],right2_new[1])

    if ( ((status) and (flag)) == 1):    
        if visited_arr[right2_new[0],right2_new[1]]==0:
            visited_arr[right2_new[0],right2_new[1]]=1
            visited_nodes.append(right2_new)
            Q.append(right2_new)
            parent_list[right2_new[0]][right2_new[1]]=current_node
            euclidean_distance_arr[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
            cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_distance_arr[right2_new[0],right2_new[1]]
        else:
            if cost[right2_new[0],right2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right2_new[0]][right2_new[1]]=current_node
                euclidean_distance_arr[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
                total_cost_arr[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_distance_arr[right2_new[0],right2_new[1]]
            
    left1_new=left1(current_node[0],current_node[1],current_node[2])
    status=boundary_check(left1_new[0],left1_new[1])
    flag=obstacle_map(left1_new[0],left1_new[1])

    if ( ((status) and (flag)) == 1):
        if visited_arr[left1_new[0],left1_new[1]]==0:
            visited_arr[left1_new[0],left1_new[1]]=1
            visited_nodes.append(left1_new)
            Q.append(left1_new)
            parent_list[left1_new[0]][left1_new[1]]=current_node
            euclidean_distance_arr[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
            cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_distance_arr[left1_new[0],left1_new[1]]
        else:
            if cost[left1_new[0],left1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                euclidean_distance_arr[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
                parent_list[left1_new[0]][left1_new[1]]=current_node
                total_cost_arr[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_distance_arr[left1_new[0],left1_new[1]]
    
    left2_new=left2(current_node[0],current_node[1],current_node[2])
    status=boundary_check(left2_new[0],left2_new[1])
    flag=obstacle_map(left2_new[0],left2_new[1])

    if ( ((status) and (flag)) == 1):
        if visited_arr[left2_new[0],left2_new[1]]==0:
            visited_arr[left2_new[0],left2_new[1]]=1
            visited_nodes.append(left2_new)
            Q.append(left2_new)
            parent_list[left2_new[0]][left2_new[1]]=current_node
            euclidean_distance_arr[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
            cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            total_cost_arr[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_distance_arr[left2_new[0],left2_new[1]]
        else:
            if cost[left2_new[0],left2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[left2_new[0]][left2_new[1]]=current_node
                euclidean_distance_arr[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
                total_cost_arr[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_distance_arr[left2_new[0],left2_new[1]]

print("Goal has been reached")


# In[17]:


start=[x_start,y_start,0]
path=[]
def path_trace(Goalbound,start):
    GN=goalbound
    path.append(goalbound)
    while (GN!=start):
        a=parent_list[GN[0]][GN[1]]
        path.append(a)
        GN=a

path_trace(goal,start)
del path[len(path)-1]
print('Path obtained',path)


# In[18]:


# Interfacing with V-Rep and giving the motions of the robot by popping from the obtained path

while path:
    p = path.pop()
    movement = p[3]
    
     # Establishing a connection with VREP through python API connection
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

    if clientID!=-1:
        print ("Remote API connection secure")
        
        errorCode, Motor_left_VREP = vrep.simxGetObjectHandle(clientID, "wheel_left_joint", vrep.simx_opmode_oneshot_wait)
        errorCode, Motor_Right_VREP = vrep.simxGetObjectHandle(clientID, "wheel_right_joint", vrep.simx_opmode_oneshot_wait)
        if movement == 'straight_0':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)
        
        elif movement == 'straight_1':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)
        
        elif movement == 'left_0':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        elif movement == 'left_1':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        elif movement == 'left_2':
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

    
        elif movement == 'right_0':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)
            
        elif movement == 'right_1':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        elif movement == 'right_2':
            vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 60*(math.pi/30), vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 30*(math.pi/30), vrep.simx_opmode_streaming)
            time.sleep(0.4)

        

vrep.simxSetJointTargetVelocity(clientID, Motor_left_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)
vrep.simxSetJointTargetVelocity(clientID, Motor_Right_VREP, 0*(math.pi/30), vrep.simx_opmode_streaming)

returnCode,handle = vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)
returnCode,position = vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking)

