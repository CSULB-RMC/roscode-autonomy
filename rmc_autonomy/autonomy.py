import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Int8, Int16MultiArray
from geometry_msgs.msg import Point

import math

# STATES FOR STATE MACHINE
# 0: need to calibrate position tracking and determine where to go
# 1: go to mining zone
# 2: in mining zone, get ready to mine/start mining
# 3: mining/wait until its finished
# 4: stop mining, get ready to move back to start
# 5: move accross back to start
# 6: deposit regolith
# 7: stop deposit and get ready to go back to state 1

class LBLState:
    next_state = 0
    func = None

    def __init__(self, next_state_init = 0, func_init = None):
        self.next_state = next_state_init
        self.func = func_init

class LBLAutonomy(Node):
    state = 0
    posBuffer = [Point()]
    rotBuffer = []
    state_info = None

    finished_moving = False
    calibrate_in_progress = False

    old_pos = Point()

    def __init__(self):
        super().__init__('LBLAutonomy')

        self.state_info = {0: LBLState(1, self.calibrate_state),
        1: LBLState(2, self.travel_to_mining()),
        2: LBLState(3, self.start_mining()),
        3: LBLState(4, self.keep_mining()),
        4: LBLState(5, self.stop_mining()),
        5: LBLState(6, self.travel_to_collector()),
        6: LBLState(7, self.deposit_regolith()),
        7: LBLState(0, self.stop_depost_regolith()) }

        #MOTOR CONTROL
        self.dt_left_pub = self.create_publisher(Int8, 'dt_left', 1)
        self.dt_right_pub = self.create_publisher(Int8, 'dt_right', 1)

        self.dumper_pub = self.create_publisher(Int8, 'dumper_control', 10)
        self.bucketladder_lifter_pub = self.create_publisher(Int8, 'bucketladder_lifter_control', 10)
        self.bucketladder_telescope_pub = self.create_publisher(Int8, 'bucketladder_telescope_control', 10)
        self.bucketladder_digger_pub = self.create_publisher(Int8, 'bucketladder_digger_control', 10)

        self.stop_all_pub = self.create_publisher(Empty, 'stop_all_arduino', 10)

        #CAMERA
        self.object_detection_sub = self.create_subscription(Int16MultiArray, 'publisher_obstruction', self.obstacle_detected_callback, 10)

        #POSITION TRACKING
        self.pos_tracking_sub = self.create_subscription(Point, 'position_tracking_pos', self.pos_tracking_callback, 10)

        #supress unused warning
        self.object_detection_sub
        self.pos_tracking_sub

        self.autonomy_timer = self.create_timer(0.1, self.autonomy_callback)

    def autonomy_callback(self):
        self.state_info[self.state].func()

        #self.stateFunc[0]()
    def timer_callback(self):
        self.finished_moving = True

    def obstacle_detected_callback(self, msg: Int16MultiArray):
        pass
        #have to reconstruct tuples from MultiArray
        
    
    def pos_tracking_callback(self, msg: Point):
        #do stuff with the point message
        self.get_logger().info("Pos tracking callback running.\n")
        self.posBuffer.append(msg)
        if len(self.posBuffer) > 5:
            self.posBuffer.pop(0)

    def calibrate_state(self): # state 0
        if self.calibrate_in_progress == False:
            self.get_logger().info("Calibration starting.\n")
            self.calibrate_in_progress = True
            #calibration has started
            #get position and direction
            self.old_pos = self.posBuffer[-1] # latest position
            
            # TO BE CHANGED TO MOVE CERTAIN AMOUNT OF METERS VIA ENCODERS
            speed = Int8()
            speed.data = 100
            self.get_logger().info("Moving rover.\n")
            self.dt_left_pub.publish(speed)
            self.dt_right_pub.publish(speed)
            self.create_timer(1, self.timer_callback)


            #get angle

            #get position
            #check if it has regolith (above a threshold), if it does, set state to 5
            #if not, set state to 1
        if self.finished_moving:
            speed = Int8()
            speed.data = 0
            self.dt_left_pub.publish(speed)
            self.dt_right_pub.publish(speed)
            self.get_logger().info("Stopping rover.\n")

            newPos = self.posBuffer[-1]
            if newPos.x - self.old_pos.x == 0:
                slope = 0
            else:
                slope = (newPos.y - self.old_pos.y) / (newPos.x - self.old_pos.x) 
            new_angle = math.degrees(math.atan(slope))
            self.get_logger().info("Angle: %d \n" % new_angle)
            self.rotBuffer.append(new_angle)

            self.state = self.state_info[self.state].next_state
            self.calibrate_in_progress = False

            #check the new position, then do inverse tangent to get angle
            

    
    def travel_to_mining(self): # state 1
        #travel across to mining zone from starting area
        pass
    
    def start_mining(self): # state 2
        #deploy bucketladder
        pass

    def keep_mining(self): # state 3
        #dig with bucketladder, check all parameters to determine when to stop

        """
        - determine if this will be the last cycle or not
        - using load cell, determine the minimum amount of regolith to get this cycle
            - if not last cycle, then make sure captured regolith is at least 0.1kg (100g)
            - if it IS the last cycle, make sure captured regolith is at least whatever is needed to add total regolith (including previous minings) is 1kg.
        - the time spent mining is weighed between two variables: how much regolith is left to mine, and a set timer. Final cycle will always try and get as much as possible
        """
        pass
    
    def stop_mining(self): # state 4
        #retract bucket ladder, get ready to move back
        pass

    def travel_to_collector(self): # state 5
        #move across to starting zone/get ready to dump in collector
        pass
    
    def deposit_regolith(self): # state 6
        #raise regcon system to dump the regolith in container
        pass
    
    def stop_depost_regolith(self): # state 7
        #lower regcon, get ready to start another cycle

        #eg: state = 1
        pass




def main(args=None):
    rclpy.init(args=args)

    lbl_autonomy = LBLAutonomy()

    rclpy.spin(lbl_autonomy)

    lbl_autonomy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()