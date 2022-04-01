import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Int8, Int16MultiArray
from geometry_msgs.msg import Point

# STATES FOR STATE MACHINE
# 0: need to calibrate position tracking and determine where to go
# 1: go to mining zone
# 2: in mining zone, get ready to mine/start mining
# 3: mining/wait until its finished
# 4: stop mining, get ready to move back to start
# 5: move accross back to start
# 6: deposit regolith
# 7: stop deposit and get ready to go back to state 1

class LBLAutonomy(Node):
    state = 0

    def __init__(self):
        super().__init__('LBLAutonomy')

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

        while 1==1:
            if self.state == 0:
                self.calibrate_state()
            elif self.state == 1:
                self.travel_to_mining()
            elif self.state == 2:
                self.start_mining()
            elif self.state == 3:
                self.keep_mining()
            elif self.state == 4:
                self.stop_mining()
            elif self.state == 5:
                self.travel_to_collector()
            elif self.state == 6:
                self.deposit_regolith()
            elif self.state == 7:
                self.stop_depost_regolith()


    def obstacle_detected_callback(self, msg: Int16MultiArray):
        if self.state == 1:
            pass
        #have to reconstruct tuples from MultiArray
        pass
    
    def pos_tracking_callback(self, msg: Point):
        #do stuff with the point message
        pass

    def calibrate_state(self): # state 0
        #get position and direction
        pass
    
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