# spiral_path_drone

In this project ArduPilot SITL and MAVProxy is used with Gazebo to simulate a drone.

Dronekit python script is used to send commands to the drone and execute the spiral path.

Finally OpenCV is used to detect the arUco marker and trigger the landing.


## Setup

In a terminal window execute the following Gazebo command

    gz sim -v4 -r iris_runway.sdf
    
In another window execute Arducopter SITL command and wait for initialization

    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
    
Finally run the DroneKit script in a third terminal

    python2 spiral_path_drone.py
