##################################################
# Homework 1, CSE 494
# Spring 2021, Dr. Heni Ben Amor
# @author Michael Drolet
# This file is used to command the Pioneer robot
# in CoppeliaSim and provide an interfaces for
# applying simple vector properties for robot
# control.
##################################################
# STUDENT NAME:Jeffrey Ciferno
# ASU ID:1217664240
##################################################

import numpy as np
import cv2
import os
from pyrep import PyRep
import time
import math

maxforce = 4.0
maxspeed = 2.0
confirmed = 0
# Simple 2D Vector class for CoppeliaSim objects
class Vector2D():
    def __init__(self, vector=[0,0]):
        self.x = vector[0]
        self.y = vector[1]

    def add(self, vec):
        # TODO: YOUR CODE HERE
        return Vector2D([self.x + vec.x, self.y + vec.y]) 

    @staticmethod # This should be used as a static method!
    def sub(vec1, vec2):
        return Vector2D([vec1.x - vec2.x, vec1.y - vec2.y])

    def limit(self, scaler = 0.5):
        # TODO: YOUR CODE HERE
        m = abs(math.sqrt((self.x*self.x) + (self.y*self.y)))
        if (m > abs(maxforce)):
            self.x = self.x*scaler
            self.y = self.y*scaler
        else:
            self.x = self.x
            self.y = self.y

        return self        

    def normalize(self):
        length = 1
        m = abs(math.sqrt((self.x*self.x) + (self.y*self.y)))
        self.x = length * self.x / m
        self.y = length * self.y / m
        return self

    def mult(self, scalar):
        self.x = self.x*scalar
        self.y = self.y*scalar

    def magnitude(self):
        # TODO: Your code here
        m = abs(math.sqrt((self.x*self.x) + (self.y*self.y)))
        return m


# Defines a class that we use to communicate with the robot
# and execute homework problems
class Vehicle():
    def __init__(self):
        # Create a remote API
        self.pyrep = PyRep()

        # Attept to open CoppeliaSim
        scene_path = os.getcwd() + "/hw1_scene.ttt"
        self.pyrep.launch(scene_path, headless=False)
        time.sleep(0.5)
        self.pyrep.start()

        # maximum speed/force parameters
        self.max_speed = maxspeed
        self.max_force = maxforce
    # Returns an array of the following object:
    # [location_vector, radius_of_cylinder]
    # type(location_vector) = Vector2D
    # type(radius_of_cylinder) = float
    def getObstacles(self):
        ints,floats,strings,byte = self.pyrep.script_call(function_name_at_script_name="getObstacles@script",
                                script_handle_or_type=1,
                                ints=(),
                                floats=(),
                                strings=(),
                                bytes ="")

        obstacles = []
        for i in range(int(len(floats)/3)):
            loc = Vector2D([floats[i*3], floats[i*3 +1]])
            obstacles.append([loc, floats[i*3 +2]])

        return obstacles

    # Returns a Vector2D specifying the location of the desired target from the origin
    def getTarget(self, target):
        if target == 'WP1':
            target_type = 1
        elif target == 'WP2':
            target_type = 2
        elif target == 'WP3':
            target_type = 3
        elif target == 'Home':
            target_type = 4
        elif target == 'PM':
            target_type = 5
        elif target == 'OF1':
            target_type = 6
        elif target == 'OF2':
            target_type = 7
        elif target == 'OF3':
            target_type = 8
        else:
            print('Invalid target name! (Look at the CoppeliaSim scene graph for valid names)')
            target_type = 0

        ints,floats,strings,byte = self.pyrep.script_call(function_name_at_script_name="getTarget@script",
                                script_handle_or_type=1,
                                ints=(),
                                floats=([target_type]),
                                strings=(),
                                bytes ="")

        return Vector2D([floats[0], floats[1]])

    # Get the location (as a Vector2D) of the pioneer robot
    def getLocation(self):
        ints,floats,strings,byte = self.pyrep.script_call(function_name_at_script_name="getLocation@script",
                                script_handle_or_type=1,
                                ints=(),
                                floats=(),
                                strings=(),
                                bytes ="")

        return Vector2D([floats[0], floats[1]])

    # Shutdown PyRep
    def shutdown(self):
        self.pyrep.shutdown()

    # Get the velocity (as a Vector2D) of the pioneer robot
    def getVelocity(self):
        ints,floats,strings,byte = self.pyrep.script_call(function_name_at_script_name="getVelocity@script",
                                script_handle_or_type=1,
                                ints=(),
                                floats=(),
                                strings=(),
                                bytes ="")

        return Vector2D([floats[0], floats[1]])

    # Apply a force vector to the pioneer robot
    def applyForce(self, vec2D):
        self.pyrep.script_call("applyForce@script",
                                1,
                                ints=(),
                                floats=([vec2D.x, vec2D.y]),
                                strings=(),
                                bytes ="")
        self.pyrep.step()

    # Seek the desired target (input Vector2D)
    # Utilize the 'options' keyword at your discretion
    # (i.e, options='arrival' to stop at target)
    def seek(self, target, options='default'):
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()
        desired.normalize()
        approach = 2.5
        if options == 'arrival':
            if(d < approach):
                desired.mult(d/(approach*maxspeed))
            else:
                desired.mult(maxspeed)
        if options == 'avoid':
                desired.mult(d/(approach*maxspeed))
                print("avoid")
        
        steer = Vector2D.sub(desired, self.getVelocity())
        steer.limit(maxforce)
        self.applyForce(steer)
        self.pyrep.step()
        # self.applyForce(steer)
        # optional: return distance to target (for arrival)


    # Arrive to the Pitchers Mound (aka Blue Disc or 'PM' target)
    # Then return back to Home ('Home')
    def executeProblem1(self):
        #SETPM
        target = self.getTarget('PM')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()

        while(d > .015):
            location = self.getLocation()
            desired = Vector2D.sub(target, location)
            d = desired.magnitude()
            self.seek(target, 'arrival')
            print(d)

        #SETHOME
        target = self.getTarget('Home')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()

        while(d > .015):
            location = self.getLocation()
            desired = Vector2D.sub(target, location)
            d = desired.magnitude()
            self.seek(target, 'arrival')
            print(d)

        # Hint: seek should be used inside of a loop! (multiple times)
        # TODO: Your Code Here

    # Round the bases! Use Path Following to navigate through the waypoints:
    # First base ('WP1'), Second base ('WP2'), Third base ('WP3'), and then Home ('Home')
    def executeProblem2(self):
        
        #SETWP1
        target = self.getTarget('WP1')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()

        while(d > .5):
            location = self.getLocation()
            desired = Vector2D.sub(target, location)
            d = desired.magnitude()
            self.seek(target, 'default')
            print(d)

        #SETWP2
        target = self.getTarget('WP2')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()

        while(d > .5):
            location = self.getLocation()
            desired = Vector2D.sub(target, location)
            d = desired.magnitude()
            self.seek(target, 'default')
            print(d)
        
        #SETWP3
        target = self.getTarget('WP3')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()

        while(d > .5):
            location = self.getLocation()
            desired = Vector2D.sub(target, location)
            d = desired.magnitude()
            self.seek(target, 'default')
            print(d)

        #SETHOME
        target = self.getTarget('Home')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()

        while(d > .5):
            location = self.getLocation()
            desired = Vector2D.sub(target, location)
            d = desired.magnitude()
            self.seek(target, 'default')
            print(d)

    # Implement Obstacle Avoidance by traveling to
    # Left Field ('OF1'), then Center Field ('OF2'), and then Right Field ('OF3')
    # Then return back to Home plate
    def executeProblem3(self):
        obstacles = self.getObstacles()
        #print(obstacles)
        target = self.getTarget('OF1')
        location = self.getLocation()
        desired = Vector2D.sub(target, location)
        d = desired.magnitude()
        while(d > .5):
            for i in obstacles:
                Obs = Vector2D.sub(i[0], location)
                obstD = Obs.magnitude()
                print("obs distance")
                print(obstD)
                if (obstD < (3 * i[1])):
                    location = self.getLocation()
                    notdesired = Vector2D.sub(location, Obs)
                    desired = Vector2D.sub(target, location)
                    adjust = desired;
                    adjust.x = -(notdesired.y - desired.y) + desired.x
                    adjust.y = (notdesired.x - desired.x) + desired.y
                    #d = notdesired.magnitude()
                    self.seek(adjust, 'avoid')

                location = self.getLocation()
                desired = Vector2D.sub(target, location)
                d = desired.magnitude()
                self.seek(target, 'default')
                print(d)

        # TODO: Your Code Here
        return None


    # Implement the Wander algorithm
    # Note the grid size is 75x75:
    # (X: -37.5 to 37.5), (Y: -37.5 to 37.5)
    def executeProblem4(self):
        # TODO: Your Code Here
        return None

# Main entry for the code
if __name__ == "__main__":
    # Create the robot wrapper
    pioneer = Vehicle()
    pioneer.executeProblem1() # Seek and Arrival
    pioneer.executeProblem2() # Path Following
    pioneer.executeProblem3() # Obstacle Avoidance
    pioneer.executeProblem4() # Wander
    pioneer.shutdown()
