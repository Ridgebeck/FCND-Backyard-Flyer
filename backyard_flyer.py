import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.square_size = 10.0

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        
        #convert altitude value of local position
        current_position = np.copy(self.local_position)
        current_position[2] = current_position[2] * -1

        #check if drone is close enough to target position
        position_reached = np.allclose(self.target_position, current_position, rtol=0.0, atol=0.25, equal_nan=False)
        if position_reached == True:
            print("target position {} reached".format(self.target_position))
            if self.flight_state == States.TAKEOFF:
                self.all_waypoints = self.calculate_box(self.target_position, self.square_size)
                print("Waypoints: {}".format(self.all_waypoints))
                self.flight_state = States.WAYPOINT
                self.waypoint_transition()
            elif self.flight_state == States.WAYPOINT:
                self.waypoint_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        #check if drone is in mission
        if not self.in_mission:
            return
        #transition based on current state
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self, start_point, size):
        waypoint_1 = np.copy(start_point)
        waypoint_1[0] = start_point[0] + size
        waypoint_2 = np.copy(waypoint_1)
        waypoint_2[1] = waypoint_1[1] + size
        waypoint_3 = np.copy(waypoint_2)
        waypoint_3[0] = waypoint_2[0] - size

        return [waypoint_1, waypoint_2, waypoint_3, start_point]

    def arming_transition(self):
        print("arming transition")
        self.take_control() #take control of the drone
        self.arm()  #arm the drone
        #set the current global position as the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        print("home position set to: {} / {} / {}".format(self.global_position[0], self.global_position[1], self.global_position[2]))
        self.flight_state = States.ARMING #change state

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0 #target altitude is 3m above ground
        self.target_position[2] = target_altitude #set target position [x,y,z]
        self.takeoff(target_altitude) #takeoff towards target position
        self.flight_state = States.TAKEOFF #change state

    def waypoint_transition(self):
        #command move to next waypoint
        print("waypoint transition")
        if self.all_waypoints:
            self.target_position = self.all_waypoints.pop(0)
            print("new target position: {}".format(self.target_position))
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
        #land after all waypoints have been reached
        else:
            print("no waypoints left")
            #check that horizontal velocity is not too high
            if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                self.landing_transition() #transition into landing

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
