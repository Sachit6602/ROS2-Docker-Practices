import cv2 
import numpy as np
from math import pow, atan2, sqrt ,  degrees, asin

class bot_motionplanning():

    def __init__(self):

         # counter to move car forward for a few iterations
        self.count = 0
        # State Variable => Initial Point Extracted?
        self.pt_i_taken = False
        # [Container] => Store Initial car location
        self.init_loc = 0

        # State Variable => Angle relation computed?
        self.angle_relation_computed = False

        # [Container] => Bot Angle [Image]
        self.bot_angle = 0
        # [Container] => Bot Angle [Simulation]
        self.bot_angle_s = 0
        # [Container] => Angle Relation Bw(Image & Simulation)
        self.bot_angle_rel = 0
        # State Variable ==> (Maze Exit) Not Reached ?
        self.goal_not_reached_flag = True
        # [Containers] ==> Mini-Goal (X,Y)
        self.goal_pose_x = 0
        self.goal_pose_y = 0
        # [Iterater] ==> Current Mini-Goal iteration
        self.path_iter = 0

        # [Containers] Previous iteration Case [Angle or Dist or Mini_Goal]
        self.prev_angle_to_turn = 0
        self.Prev_distance_to_goal = 0
        self.prev_path_iter = 0

        # [Iterators] Iterations elapsed since no change or backpeddling
        self.angle_not_changed = 0
        self.dist_not_changed = 0
        self.goal_not_changed =0
        self.goal_not_changed_long =0
        self.backpeddling = 0

        # [State Variables] Stuck on Wall? Ready to backpeddle
        self.trigger_backpeddling = False
        # [State Variables] Can't reach goal? Try next appropriate one
        self.trigger_nxtpt = False

        self.curr_speed = 0
        self.curr_angle = 0

    
    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians


    @staticmethod
    def angle_n_dist(pt_a,pt_b):
        # Trignometric rules Work Considering.... 
        #
        #       [ Simulation/Normal Convention ]      [ Image ]
        #
        #                    Y                    
        #                     |                     
        #                     |___                     ____ 
        #                          X                  |     X
        #                                             |
        #                                           Y
        #
        # Solution: To apply same rules , we subtract the (first) point Y axis with (Second) point Y axis
        error_x = pt_b[0] - pt_a[0]
        error_y = pt_a[1] - pt_b[1]

        # Calculating distance between two points
        distance = sqrt(pow( (error_x),2 ) + pow( (error_y),2 ) )

        # Calculating angle between two points [Output : [-Pi,Pi]]
        angle = atan2(error_y,error_x)
        # Converting angle from radians to degrees
        angle_deg = degrees(angle)

        if (angle_deg>0):
            return (angle_deg),distance
        else:
            # -160 +360 = 200, -180 +360 = 180,  -90 + 360 = 270
            return (angle_deg + 360),distance
        
        #             Angle bw Points 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]

    def get_pose(self,data):

        # We get the bot_turn_angle in simulation Using same method as Gotogoal.py
        quaternions = data.pose.pose.orientation
        (roll,pitch,yaw)=self.euler_from_quaternion(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
        yaw_deg = degrees(yaw)

        # [Maintaining the Consistency in Angle Range]
        if (yaw_deg>0):
            self.bot_angle_s = yaw_deg
        else:
            # -160 + 360 = 200, -180 + 360 = 180 . -90 + 360 = 270
            self.bot_angle_s = yaw_deg + 360
        
        #              Bot Rotation 
        #      (OLD)        =>      (NEW) 
        #   [-180,180]             [0,360]

    def nav_path(self,bot_loc,path,velocity,velocity_publisher):
        # Check if path is valid and not empty
        if not path or len(path) == 0:
            print("Error: No valid path found")
            velocity.linear.x = 0.0
            velocity_publisher.publish(velocity)
            return

        # If valid path found
        if (type(path)!=int):
            # Reset path iteration if needed
            if self.path_iter >= len(path):
                self.path_iter = 0
                
            # Trying to reach first mini-goal
            if (self.path_iter==0):
                self.goal_pose_x = path[self.path_iter][0]
                self.goal_pose_y = path[self.path_iter][1]

        print(f"Debug: count={self.count}, angle_relation_computed={self.angle_relation_computed}")

        if (self.count >20):
            if not self.angle_relation_computed:
                print("Debug: Computing angle relation")
                velocity.linear.x = 0.0
                # Stopping our car
                velocity_publisher.publish(velocity)
                # Extracting Car angle (Img) from car_InitLoc and car_FinalLoc after moving forward (50 iters)
                self.bot_angle, _= self.angle_n_dist(self.init_loc, bot_loc)
                self.bot_angle_init = self.bot_angle
                # Finding relation coeffiecient between car_angle (Image <-> Simulation)
                self.bot_angle_rel = self.bot_angle_s - self.bot_angle
                self.angle_relation_computed = True
            else:
                # [For noob luqman] : Extracting Car angle [From Simulation angle & S-I Relation]
                self.bot_angle = self.bot_angle_s - self.bot_angle_rel

                print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car angle (Simulation) = {}".format(self.bot_angle,self.bot_angle_rel,self.bot_angle_s))
                print("Car angle_Initial (Image) = ",self.bot_angle_init)
                print("Car loc {}".format(bot_loc))

        else:
            # If bot initial location not already taken
            if not self.pt_i_taken:
                print("Debug: Taking initial point")
                # Set init_loc = Current bot location
                self.init_loc = bot_loc
                self.pt_i_taken = True
                
            # Keep moving forward for 20 iterations(count)
            velocity.linear.x = 1.0
            velocity_publisher.publish(velocity)
            self.count += 1
        

        
