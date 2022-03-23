import numpy as np

from DroneClient import DroneClient
import time
import math
import airsim.utils


def print_(string):
    debug = 1
    if debug:
        print(string)


DEST_ARRIVED_DIST = 4
SLOWING_DIST = 7
MAX_SPEED = 5
MAX_TIME = 0.62
WALL_SPEED = 5
WALL_WAIT = 0.3
WAIT_LIDAR = 0.7
STOP_THRESH = 35
STOP_THRESH_WALL = 15
WAIT_SPIN = 2
BETA = 0.5  # rad
LEFT_ANGLE = 0.2

SIGHT_ANGLE = 0.6
SIGHT_ANGLE_WALL = 0.5

DEFAULT_FLY_DIST = 15
DEFAULT_FLY_WAIT = 1
CLEAR_FLY_DIST = 20
CLEAR_FLY_WAIT = 10
MIN_WALL_DIST_Y = 0.2
MIN_WALL_DIST_X = 1
GO_BACK_DIST = 5
LIDAR_SAMPLES = 4
LIDAR_SAMPLES_WAIT = 0.5


class DroneControl:
    def __init__(self, drone_client, final_dest):
        self.drone = drone_client
        self.final_dest = final_dest
        self.current_speed = 0
        self.x_direction, self.y_direction = self.get_direction(final_dest)
        self.last_wall_obs = tuple()
        self.last_xsgin = 1
        self.last_ysgin = 1
        self.last_pos = None

    def get_direction(self, final_dest):
        curr_pos = self.drone.getPose()
        curr_x = curr_pos.pos.x_m
        curr_y = curr_pos.pos.y_m
        x_dir = math.copysign(1, final_dest[0] - curr_x)
        y_dir = math.copysign(1, final_dest[1] - curr_y)
        return x_dir, y_dir

    def calcRx(self, rad):
        cos_deg = math.cos(rad)
        sin_deg = math.sin(rad)
        RX = np.array([[1, 0, 0],
                       [0, cos_deg, sin_deg],
                       [0, -sin_deg, cos_deg]])
        return RX

    def calcRy(self, rad):
        cos_deg = math.cos(rad)
        sin_deg = math.sin(rad)
        RY = np.array([[cos_deg, 0, -sin_deg],
                       [0, 1, 0],
                       [sin_deg, 0, cos_deg]])
        return RY

    def calcRz(self, rad):
        cos_deg = math.cos(rad)
        sin_deg = math.sin(rad)
        RZ = np.array([[cos_deg, sin_deg, 0],
                       [-sin_deg, cos_deg, 0],
                       [0, 0, 1]])
        return RZ

    def getRotMatrix(self, pos):
        rx = self.calcRx(pos.orientation.x_rad)
        ry = self.calcRy(pos.orientation.y_rad)
        rz = self.calcRz(pos.orientation.z_rad)
        # print("rx: ")
        # print(rx)
        # print("ry: ")
        # print(ry)
        # print("rz: ")
        # print(rz)
        # print(np.matmul(ry, rz))
        rot_matrix = np.matmul(rx, np.matmul(ry, rz))
        rot_matrix = np.linalg.inv(rot_matrix)
        return rot_matrix

    def getTranslationMatrix(self):
        pos = self.drone.getPose()
        rot_matrix = self.getRotMatrix(pos)
        trans_matrix = [[*rot_matrix[0], pos.pos.x_m],
                        [*rot_matrix[1], pos.pos.y_m],
                        [*rot_matrix[2], pos.pos.z_m],
                        [0, 0, 0, 1]]
        return trans_matrix

    def droneP2GlobalP(self, drone_point, curr_pos):
        print_("droneP2GlobalP")
        drone_point = np.array([drone_point[0], drone_point[1], drone_point[2]])
        # print("pos")
        # print(curr_pos)
        # print("drone_point")
        # print(drone_point)
        rot_matrix = self.getRotMatrix(curr_pos)
        global_point = np.matmul(rot_matrix, drone_point) + np.array(
            [curr_pos.pos.x_m, curr_pos.pos.y_m, curr_pos.pos.z_m])
        # print("R*P")
        # print(np.matmul(rot_matrix, drone_point))
        # print("global_point")
        # print(global_point)
        return global_point

    def getDistToFinalDest(self):
        curr_pos = self.drone.getPose()
        curr_x = curr_pos.pos.x_m
        curr_y = curr_pos.pos.y_m
        dest_x = self.final_dest[0]
        dest_y = self.final_dest[1]
        dist_left = math.sqrt((dest_x - curr_x) ** 2 + (dest_y - curr_y) ** 2)
        return dist_left

    def slowdown(self):
        return self.current_speed / 2

    def getMaxSpeed(self, curr_dest, wall=False):
        dist_left = self.getDistToFinalDest()
        max_possible_speed = dist_left / MAX_TIME
        # print(curr_dest)
        # print(self.final_dest)
        if tuple(curr_dest) == self.final_dest and dist_left < SLOWING_DIST:
            self.current_speed = self.slowdown()
        elif wall:
            self.current_speed = WALL_SPEED
        else:
            self.current_speed = max_possible_speed
        # print("max possible speed")
        # print(max_possible_speed)
        self.current_speed = min(MAX_SPEED, self.current_speed, max_possible_speed)
        # print("chosen speed")
        # print(self.current_speed)
        return self.current_speed

    def ReachToFinalDest(self):
        print("ReachToFinalDest")
        dist_left = self.getDistToFinalDest()
        if dist_left <= DEST_ARRIVED_DIST:
            return True
        return False

    def getDistToDest(self, dest):
        curr_pos = self.drone.getPose()
        curr_x = curr_pos.pos.x_m
        curr_y = curr_pos.pos.y_m
        dest_x = dest[0]
        dest_y = dest[1]
        dist_left = math.sqrt((dest_x - curr_x) ** 2 + (dest_y - curr_y) ** 2)
        return dist_left

    def goToDest(self, destination):
        print("goToDest")
        while not self.reachToDest(destination):
            time.sleep(WALL_WAIT)
            is_obs, _ = self.isObsInSight(wall=True)
            if is_obs:
                break

    def reachToDest(self, curr_dest):
        print_("reachToDest")
        dist_left = self.getDistToDest(curr_dest)
        print_("dist left")
        print_(dist_left)
        if dist_left <= DEST_ARRIVED_DIST:
            return True
        return False

    def ClearPathToFinalDest(self, no_speed=False):
        print("ClearPathToFinalDest")
        if no_speed:
            self.current_speed = 0
        self.drone.flyToPosition(*self.final_dest, self.current_speed)
        is_obs, _ = self.isObsInSight()
        return (not is_obs), _

    def flyToFinalDest(self):
        print("flyToFinalDest")
        if self.ReachToFinalDest():
            return True
        clear_path, _ = self.ClearPathToFinalDest()
        if clear_path:
            print("There is a Clear path to final dest")
            self.drone.flyToPosition(*self.final_dest, self.getMaxSpeed(self.final_dest))
            self.goToDest(self.final_dest)
        else:
            print("Wall detected")
            self.followTheWall(True)

    def goBack(self, x, y):
        print("goBack")
        back_vec = (-(math.copysign(1, x) * GO_BACK_DIST + x), -(math.copysign(1, y) * GO_BACK_DIST + y), 0)
        point = controller.droneP2GlobalP(back_vec, self.drone.getPose())
        speed = self.getMaxSpeed(point, wall=True)
        self.drone.flyToPosition(*point, speed)
        # self.goToDest(point)
        time.sleep(DEFAULT_FLY_WAIT)
        back_vec = (-GO_BACK_DIST, 0, 0)
        point = controller.droneP2GlobalP(back_vec, self.drone.getPose())
        self.drone.flyToPosition(*point, 0)
        time.sleep(WAIT_SPIN)

    def getRightVec(self):
        return DEFAULT_FLY_DIST * math.sin(BETA), -DEFAULT_FLY_DIST * math.cos(BETA), 0

    def isObsInSight(self, wall=False):
        for i in range(LIDAR_SAMPLES):
            print_("isObsInSightLoop")
            time.sleep(LIDAR_SAMPLES_WAIT)
            lid_data = self.drone.getLidarData().points
            if len(lid_data) < 3:
                continue
            for i in range(0, len(lid_data), 3):
                x = lid_data[i]
                y = lid_data[i + 1]
                print_("atan")
                print_(abs(math.atan(abs(y / x))))
                dist = math.sqrt(x ** 2 + y ** 2)
                print_("dist")
                print_(dist)
                if dist < MIN_WALL_DIST_X:
                    print("obs in fly path1")
                    self.goBack(x, y)
                    # lid_data = self.drone.getLidarData().points
                    # if len(lid_data) < 3:
                    #     return False, None, None
                    # x = lid_data[0]
                    # y = lid_data[1]
                    return True, True
                if wall:
                    if abs(math.atan(abs(y / x))) <= SIGHT_ANGLE_WALL and dist < STOP_THRESH_WALL:
                        print("obs in fly path2")
                        pos = controller.drone.getPose()
                        point = controller.droneP2GlobalP((DEFAULT_FLY_DIST, 0, 0), pos)
                        self.drone.flyToPosition(*point, 0)
                        time.sleep(WAIT_SPIN)
                        return True, False
                else:
                    if abs(math.atan(abs(y / x))) <= SIGHT_ANGLE and dist < STOP_THRESH:
                        print("obs in fly path3")
                        pos = controller.drone.getPose()
                        point = controller.droneP2GlobalP((DEFAULT_FLY_DIST, 0, 0), pos)
                        self.drone.flyToPosition(*point, 0)
                        time.sleep(WAIT_SPIN)
                        return True, False
        return False, False

    def turnRight(self):
        print("turnRight")
        time.sleep(WAIT_LIDAR)
        turn_counter = 0
        is_obs, is_back = self.isObsInSight(wall=True)
        while is_obs:
            print("turnRightLoop")
            right_vec = self.getRightVec()
            # time.sleep(5)  # debug
            pos = controller.drone.getPose()
            point = controller.droneP2GlobalP(right_vec, pos)
            self.drone.flyToPosition(*point, 0)
            time.sleep(WAIT_SPIN)
            turn_counter = turn_counter + 1
            is_obs, is_back = self.isObsInSight(wall=True)
        print_("NUM OF TURNS")
        print_(turn_counter)
        return turn_counter, is_back

    def getLeftVec(self, angle):
        return DEFAULT_FLY_DIST * math.sin(angle), DEFAULT_FLY_DIST * math.cos(angle), 0

    def turnLeft(self, num_of_turns):
        print("turnLeft")
        left_vec = self.getLeftVec(LEFT_ANGLE)
        pos = controller.drone.getPose()
        point = controller.droneP2GlobalP(left_vec, pos)
        self.drone.flyToPosition(*point, 0)
        time.sleep(WAIT_SPIN)

    def followTheWall(self, is_obs):
        time.sleep(WAIT_LIDAR)
        while is_obs:
            print("followTheWallLoop")
            num_of_turns, is_back = self.turnRight()
            pos = self.drone.getPose()
            glb_point = self.droneP2GlobalP((CLEAR_FLY_DIST, 0, 0), pos)
            speed = self.getMaxSpeed(glb_point, wall=True)
            self.drone.flyToPosition(*glb_point, speed)
            print("curr pos:")
            print(pos)
            print("flying to: ")
            print(glb_point)
            print("with speed: ")
            print(speed)
            self.goToDest(glb_point)
            if num_of_turns and not is_back:
                self.turnLeft(num_of_turns)
            is_obs, is_back = self.isObsInSight(wall=True)
        pos = self.drone.getPose()
        glb_point = self.droneP2GlobalP((DEFAULT_FLY_DIST, 0, 0), pos)
        speed = self.getMaxSpeed(glb_point)
        self.drone.flyToPosition(*glb_point, speed)
        self.goToDest(glb_point)


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-300, -700, -100)

    time.sleep(3)
    dest = (-400, -800, -100)
    controller = DroneControl(client, dest)
    time.sleep(3)
    reach_to_dest = False
    loop_counter = 0

    #
    # dest = (12,0,0)
    # point = controller.droneP2GlobalP(dest,controller.drone.getPose())
    # print("*"*20)
    # print("pos")
    # print(client.getPose())
    # print("glb point")
    # print(point)
    # controller.drone.flyToPosition(*point, 0)
    # time.sleep(5)
    # dest = (-12,0,0)
    # point = controller.droneP2GlobalP(dest,controller.drone.getPose())
    # print("*"*20)
    # print("pos")
    # print(client.getPose())
    # print("glb point")
    # print(point)
    # controller.drone.flyToPosition(*point, 0)
    # time.sleep(5)

    while True:
        print("Roundddddd: " + str(loop_counter))
        reach_to_dest = controller.flyToFinalDest()
        if reach_to_dest:
            break

        # time.sleep(wait_time)
        loop_counter = loop_counter + 1
