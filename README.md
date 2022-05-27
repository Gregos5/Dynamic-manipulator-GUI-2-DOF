# GUI: Animation of dynamic controlled 2 DOF RR planar manipulator 
Simulation of dynamic 2-DOF RR manipulator on a python graphical user interface
all codes are on written for Python 3.9 or above.


# How to run?
Need a python 3.9 compiler or IDE to run these codes.

Libraries to install: 

Example command(py 3.9):" >pip install Numpy "
	
	Numpy : mathematics
	math : constants, mathematics
	Pygame : Graphical user interface
	Matplotlib : plot data accuratly (for further work purpuses)
	
The main code to run is named GUI_game.py. Running this code will make a window pop which is designed from the Pygame module

there are two big functions that are free to be modified: 
  
	nametoXY.py: returns set of cartesian positions for alphabet letters
	arm_para.py: returns default values for the arm lenghts, masses of the tips, gravity
	
  
# End the program ?
click twice on escape key or close the window completely to terminate the program.

# Important Note!!
the current window size is set to fit a 15.6 inch screen at least. The code still requieres some adaptation to fit any screen.
If you screen is smaller make sure to know how to quit the game using the escape key from the Compiler itself (Command window: CRLT+C)

# Interface examples (27/05)
- Menu page
three options to choose from: free, control, game
![image](https://user-images.githubusercontent.com/78451671/170700946-5c180576-24bb-41eb-96a3-acd5ec35bc8c.png)

- free Mode
Simulate arm in free motion
![image](https://user-images.githubusercontent.com/78451671/170700883-71592174-4cb0-4b5b-860a-2532c7494f27.png)

- Control mode
PD controllers, tuning perform multiple joint space action (position only)

waypoints example 
![image](https://user-images.githubusercontent.com/78451671/170699583-2c106683-6c6d-4a0c-a765-1a25145e1e70.png)

name example
![image](https://user-images.githubusercontent.com/78451671/170700396-1559d966-8533-4fa9-8d9d-19757a2c6a00.png)

-Game mode
complete tasks and tune the controller!
![image](https://user-images.githubusercontent.com/78451671/170700532-cbf989d9-2a6c-42f9-80b4-12e337f30066.png)


