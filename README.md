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
