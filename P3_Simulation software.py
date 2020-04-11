#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
import numpy as np
import cv2 as cv
from scipy.spatial import distance



# In[2]:


#measurements in centimeters
r=3.8
l=23
dt=0.5
d=20


# In[3]:
#obstacle map

def obs_map(x,y):
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


# In[4]:


obstacle_map_arr = np.ones((1000,1000,3),np.uint8)*255

for x in range(0,1000):
    for y in range(0,1000):
        if obs_map(x,y) == 0:
            obstacle_map_arr[y,x] = (0,0,0)


# In[5]:
#boundary value check

def check_boundary(x,y):
    if (x<d or x>999-d or y<d or y>999-d ):
        return 0
    else:
        return 1


# In[6]:
# Creating map list to store parent details

parent_list=[]
for j in range (1000):
    column=[]
    for i in range (1000):
        column.append(0)
    parent_list.append(column)


# In[7]:
#Getting user inputs for start points

x_start=int(input("Enter x coordinate start point "))
y_start=int(input("Enter  y coordinate start point"))

start_obs=obs_map(x_start,y_start)
start_boundary=check_boundary(x_start,y_start)

while( ((start_obs) and (start_boundary)) !=1):
    print("Incorrect start point . . Enter a valid start point")
    x_start=int(input("Enter x coordinate start point "))
    y_start=int(input("Enter y coordinate start point "))
    start_obs=obs_map(x_start,y_start)
    start_boundary=check_boundary(x_start,y_start)


start=[x_start,y_start]


# In[8]:
#Getting user inputs for goal points

x_goal=int(input("Enter  x-coordinate goal position"))
y_goal=int(input("Enter  y-coordinate goal position"))

goal_obs=obs_map(x_goal,y_goal)
goal_boundary=check_boundary(x_goal,y_goal)

while( ((goal_obs) and (goal_boundary)) !=1):
    print("Incorrect goal point . .Enter a valid goal point")
    x_goal=int(input("Enter another x-coordinate goal position"))
    y_goal=int(input("Enter another y-coordinate goal position"))
    goal_obs=obs_map(x_goal,y_goal)
    goal_boundary=check_boundary(x_goal,y_goal)

goal=[x_goal,y_goal]


# In[9]:


def Goal(x,y):
    if (x-goal[0])**2+(y-goal[1])**2-40<=0:
        return 1
    else:
        return 0


# In[10]:


cost=np.array(np.ones((1000,1000)) * np.inf)
euclidean_array=np.array(np.ones((1000,1000)) * np.inf)
totalcost=np.array(np.ones((1000,1000)) * np.inf)
visited=np.array(np.zeros((1000,1000)))


# In[11]:
#Appending the nodes



Q=[]
Q.append([x_start,y_start,0])
cost[x_start][y_start]=0
totalcost[x_start][y_start]=0

def Priority_Pop(Q):
    index_min=0
    
    X_min = Q[0][0] 
    Y_min = Q[0][1]
    for i in range(len(Q)):
        x = Q[i][0]
        y = Q[i][1]
        if totalcost[x,y] < totalcost[X_min,Y_min]:
            index_min = i
            X_min = x 
            Y_min= y
    current_node = Q[index_min]
    Q.remove(Q[index_min])
    return current_node


# In[12]:


RPM1=60
RPM2=30

#rpm to radians per 
r1=(2*22*RPM1)/(60*7)   
r2=(2*22*RPM2)/(60*7)


# In[13]:
#drive constraints

def Differential_motion(r1,r2,theta):
    dtheta=(r*(r1-r2)*dt)/l + theta
    dx=(r*(r1+r2)*math.cos(dtheta)*dt)/2
    dy=(r*(r1+r2)*math.sin(dtheta)*dt)/2
    return dtheta,dx,dy


# In[14]:
#Turtle bot movements



def straight(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def straight_faster(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def right(i,j,theta):
    dtheta,dx,dy=Differential_motion(r1,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def right1(i,j,theta):
    dtheta,dx,dy=Differential_motion(r2,0,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def right2(i,j,theta):
    if r1>r2:
        dtheta,dx,dy=Differential_motion(r1,r2,theta)
    else:
        dtheta,dx,dy=Differential_motion(r2,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def left(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r1,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def left1(i,j,theta):
    dtheta,dx,dy=Differential_motion(0,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node

def left2(i,j,theta):
    if r1>r2:
        dtheta,dx,dy=Differential_motion(r2,r1,theta)
    else:
        dtheta,dx,dy=Differential_motion(r1,r2,theta)
    new_node=(int(round(i+dx)),int(round(j+dy)),int(round(dtheta)))
    return new_node


# In[15]:


visited_node=[]
current_node=[x_start,y_start,0]
while True: 
    if Goal(current_node[0],current_node[1])==1:
        goalbound=current_node
        break
    current_node=Priority_Pop(Q)
    straight_new=straight (current_node[0],current_node[1],current_node[2])
    status=check_boundary(straight_new[0],straight_new[1])
    flag=obs_map(straight_new[0],straight_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[straight_new[0],straight_new[1]]==0:
            visited[straight_new[0],straight_new[1]]=1
            visited_node.append(straight_new)
            Q.append(straight_new)
            parent_list[straight_new[0]][straight_new[1]]=current_node
            
            cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_array[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
            totalcost[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_array[straight_new[0],straight_new[1]]
        else:
            if cost[straight_new[0],straight_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_new[0],straight_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_array[straight_new[0],straight_new[1]]=math.sqrt(math.pow((straight_new[0]-goal[0]),2) + math.pow((straight_new[1]-goal[1]),2))
                parent_list[straight_new[0]][straight_new[1]]=current_node
                totalcost[straight_new[0],straight_new[1]]= cost[straight_new[0],straight_new[1]]+euclidean_array[straight_new[0],straight_new[1]]
    
    
    straight_faster_new=straight_faster(current_node[0],current_node[1],current_node[2])
    status=check_boundary(straight_faster_new[0],straight_faster_new[1])
    flag=obs_map(straight_faster_new[0],straight_faster_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[straight_faster_new[0],straight_faster_new[1]]==0:
            visited[straight_faster_new[0],straight_faster_new[1]]=1
            visited_node.append(straight_faster_new)
            Q.append(straight_faster_new)
            parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
            cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_array[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
            totalcost[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_array[straight_faster_new[0],straight_faster_new[1]]
        else:
            if cost[straight_faster_new[0],straight_faster_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[straight_faster_new[0],straight_faster_new[1]]=(cost[current_node[0],current_node[1]]+1)
                euclidean_array[straight_faster_new[0],straight_faster_new[1]]=math.sqrt(math.pow((straight_faster_new[0]-goal[0]),2) + math.pow((straight_faster_new[1]-goal[1]),2))
                parent_list[straight_faster_new[0]][straight_faster_new[1]]=current_node
                totalcost[straight_faster_new[0],straight_faster_new[1]]= cost[straight_faster_new[0],straight_faster_new[1]]+euclidean_array[straight_faster_new[0],straight_faster_new[1]]
    
    right_new=right(current_node[0],current_node[1],current_node[2])
    status=check_boundary(right_new[0],right_new[1])
    flag=obs_map(straight_new[0],straight_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[right_new[0],right_new[1]]==0:
            visited[right_new[0],right_new[1]]=1
            visited_node.append(right_new)
            Q.append(right_new)         
            parent_list[right_new[0]][right_new[1]]=current_node
            cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
            euclidean_array[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
            totalcost[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_array[right_new[0],right_new[1]]

        else:
            if cost[right_new[0],right_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[right_new[0],right_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[right_new[0]][right_new[1]]=current_node
                euclidean_array[right_new[0],right_new[1]]=math.sqrt(math.pow((right_new[0]-goal[0]),2) + math.pow((right_new[1]-goal[1]),2))
                totalcost[right_new[0],right_new[1]]= cost[right_new[0],right_new[1]]+euclidean_array[right_new[0],right_new[1]]
    
    
    
    left_new=left(current_node[0],current_node[1],current_node[2])
    status=check_boundary(left_new[0],left_new[1])
    flag=obs_map(left_new[0],left_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[left_new[0],left_new[1]]==0:
            visited[left_new[0],left_new[1]]=1
            visited_node.append(left_new)
            Q.append(left_new)
            parent_list[left_new[0]][left_new[1]]=current_node
            euclidean_array[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
            cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
            totalcost[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_array[left_new[0],left_new[1]]
        else:
            if cost[left_new[0],left_new[1]]>(cost[current_node[0],current_node[1]]+1):
                cost[left_new[0],left_new[1]]=(cost[current_node[0],current_node[1]]+1)
                parent_list[left_new[0]][left_new[1]]=current_node
                euclidean_array[left_new[0],left_new[1]]=math.sqrt(math.pow((left_new[0]-goal[0]),2) + math.pow((left_new[1]-goal[1]),2))
                totalcost[left_new[0],left_new[1]]= cost[left_new[0],left_new[1]]+euclidean_array[left_new[0],left_new[1]]
    
    
    right1_new=right1(current_node[0],current_node[1],current_node[2])
    status=check_boundary(right1_new[0],right1_new[1])
    flag=obs_map(right1_new[0],right1_new[1])
    if ( ((status) and (flag)) == 1):   
        if visited[right1_new[0],right1_new[1]]==0:
            visited[right1_new[0],right1_new[1]]=1
            visited_node.append(right1_new)
            Q.append(right1_new)
            parent_list[right1_new[0]][right1_new[1]]=current_node
            euclidean_array[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
            cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_array[right1_new[0],right1_new[1]]
        else:
            if cost[right1_new[0],right1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right1_new[0],right1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right1_new[0]][right1_new[1]]=current_node
                euclidean_array[right1_new[0],right1_new[1]]=math.sqrt(math.pow((right1_new[0]-goal[0]),2) + math.pow((right1_new[1]-goal[1]),2))
                totalcost[right1_new[0],right1_new[1]]= cost[right1_new[0],right1_new[1]]+euclidean_array[right1_new[0],right1_new[1]]
    
    
    right2_new=right2(current_node[0],current_node[1],current_node[2])
    status=check_boundary(right2_new[0],right2_new[1])
    flag=obs_map(right2_new[0],right2_new[1])
    if ( ((status) and (flag)) == 1):    
        if visited[right2_new[0],right2_new[1]]==0:
            visited[right2_new[0],right2_new[1]]=1
            visited_node.append(right2_new)
            Q.append(right2_new)
            parent_list[right2_new[0]][right2_new[1]]=current_node
            euclidean_array[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
            cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_array[right2_new[0],right2_new[1]]
        else:
            if cost[right2_new[0],right2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[right2_new[0],right2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[right2_new[0]][right2_new[1]]=current_node
                euclidean_array[right2_new[0],right2_new[1]]=math.sqrt(math.pow((right2_new[0]-goal[0]),2) + math.pow((right2_new[1]-goal[1]),2))
                totalcost[right2_new[0],right2_new[1]]= cost[right2_new[0],right2_new[1]]+euclidean_array[right2_new[0],right2_new[1]]
            
    left1_new=left1(current_node[0],current_node[1],current_node[2])
    status=check_boundary(left1_new[0],left1_new[1])
    flag=obs_map(left1_new[0],left1_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[left1_new[0],left1_new[1]]==0:
            visited[left1_new[0],left1_new[1]]=1
            visited_node.append(left1_new)
            Q.append(left1_new)
            parent_list[left1_new[0]][left1_new[1]]=current_node
            euclidean_array[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
            cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_array[left1_new[0],left1_new[1]]
        else:
            if cost[left1_new[0],left1_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left1_new[0],left1_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                euclidean_array[left1_new[0],left1_new[1]]=math.sqrt(math.pow((left1_new[0]-goal[0]),2) + math.pow((left1_new[1]-goal[1]),2))
                parent_list[left1_new[0]][left1_new[1]]=current_node
                totalcost[left1_new[0],left1_new[1]]= cost[left1_new[0],left1_new[1]]+euclidean_array[left1_new[0],left1_new[1]]
    
    left2_new=left2(current_node[0],current_node[1],current_node[2])
    status=check_boundary(left2_new[0],left2_new[1])
    flag=obs_map(left2_new[0],left2_new[1])
    if ( ((status) and (flag)) == 1):
        if visited[left2_new[0],left2_new[1]]==0:
            visited[left2_new[0],left2_new[1]]=1
            visited_node.append(left2_new)
            Q.append(left2_new)
            parent_list[left2_new[0]][left2_new[1]]=current_node
            euclidean_array[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
            cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
            totalcost[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_array[left2_new[0],left2_new[1]]
        else:
            if cost[left2_new[0],left2_new[1]]>(cost[current_node[0],current_node[1]]+math.sqrt(2)):
                cost[left2_new[0],left2_new[1]]=(cost[current_node[0],current_node[1]]+math.sqrt(2))
                parent_list[left2_new[0]][left2_new[1]]=current_node
                euclidean_array[left2_new[0],left2_new[1]]=math.sqrt(math.pow((left2_new[0]-goal[0]),2) + math.pow((left2_new[1]-goal[1]),2))
                totalcost[left2_new[0],left2_new[1]]= cost[left2_new[0],left2_new[1]]+euclidean_array[left2_new[0],left2_new[1]]

print("The Goal is reached")


# In[16]:


start=[x_start,y_start,0]
path=[]
def path_find(Goalbound,start):
    GN=goalbound
    print('GN',GN)
    path.append(goalbound)
    while (GN!=start):
        a=parent_list[GN[0]][GN[1]]
        path.append(a)
        GN=a

path_find(goal,start)
print('path obtained',path)


# In[17]:




cv.circle(obstacle_map_arr,(int(goal[0]),int(goal[1])), (1), (0,0,255), -1);
cv.circle(obstacle_map_arr,(int(start[0]),int(start[1])), (1), (0,0,255), -1);



for i in visited_node:
    cv.circle(obstacle_map_arr,(int(i[0]),int(i[1])), (1), (255,0,0));
    output_img=cv.resize(obstacle_map_arr,None,fx=1,fy=1)
    cv.imshow('map',output_img)
    cv.waitKey(1)

for i in path:
    cv.circle(obstacle_map_arr,(int(i[0]),int(i[1])), (1), (0,0,255));
    output_img=cv.resize(obstacle_map_arr,None,fx=1,fy=1)
    cv.imshow('map',output_img)
    cv.waitKey(1)
    

cv.imwrite('Output.png',obstacle_map_arr)


cv.waitKey(0) 
cv.destroyAllWindows()


