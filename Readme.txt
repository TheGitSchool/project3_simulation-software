
- Python libraries to be imported

for P3_simulation software.py
math
numpy
distance

For P3_Vrep.py
math
numpy
distance
time
vrep

-Vrep setup

"vrep.py" and "vrepConst.py" should be present on the same folder as source code. files can be found at (programming\remoteApiBindings\python\python).

Check for 32-bit or 64-bit version of python and place the same version of vrep "remoteApi.dll" in the folder of source code file. Can be found from ("programming\remoteApiBindings\lib\lib")

V-REP map can be used from (VREP_map.ttt)

Turtlebot 2 can be included by adding the model file (Turtlebot2.ttm) inside the main folder of V-REP

The output file tested has input (50,50) and goal (600,600)

-Running the code

for phase 3 non holonomic problem
Run "P3_simulation software.py" using the terminal from the same folder location as file
The user will be asked to set the start and end point coordinates
The animation of the map will be generated.
The respective plot will also be stored in the same folder as .py files.

for Vrep
Start the vrep simulation.
Then run the P3_vrep.py file
The user will be asked for goal points.

*i was able to run the robot initially, but after an update, i couldn't get the robot to run, 
