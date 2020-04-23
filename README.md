# Object Follower
The purpose of this project is to create a program to control a turtlebot3 in simulation
the main idea is :
- the turtlebot3 follows an object
- when the object stops for 3seconds, the turtlebot3 should go home avoiding obstacle and choosing the best way

# What is done :
- control the turtlebot3 to follow an object (test needed)
- turtlebot can explore using explorer.py

# what to do :
- improve explorer's speed
- make sure time is well done (listener.py)

# installation
to use explorer.py, make sure you have TurtleBotMap in your src folder
install scipy using "python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose"
usefull : https://scipy.org/install.html#pip-install

# how to use :
- If you would like to explore the map and to make a slam, you can use explorer.py (make sure you installed scipy cf. installation and you downloaded TurtleBotMap in your catkin_ws/src)
- If you would like to follow an object, please use listerner.py (make sure you installed scipy and downloaded explorer.py and TurtleBotMap.py)
- If you would like to have a fully autonomus robot, you can use final.py (same recommendations as behind)
