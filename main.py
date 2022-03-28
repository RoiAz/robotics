import numpy as np

from DroneClient import DroneClient
import time
import math
import airsim.utils

DEBUG = 1


def print_(string):
    if DEBUG:
        print(string)


MIN_TRI_AREA = 10
DEST_ARRIVED_DIST = 4
SLOWING_DIST = 7
MAX_SPEED = 10
GLOBAL_MAX_SPEED = 12
MAX_TIME = 0.62
WALL_SPEED = 7
WALL_WAIT = 0.1
STOP_THRESH = 35
STOP_THRESH_WALL = 20
WAIT_SPIN = 0.2
WAIT_SPINL = 0.4
BETA = 0.3  # rad
LEFT_ANGLE = 0.7

SIGHT_ANGLE = 0.6
SIGHT_ANGLE_WALL = 0.4

DEFAULT_FLY_DIST = 15
DEFAULT_FLY_WAIT = 1
CLEAR_FLY_DIST = 20
CLEAR_FLY_WAIT = 10
MIN_WALL_DIST_Y = 0.2
MIN_WALL_DIST_Z = 0.5
GO_UP_MIN = 3
MIN_WALL_DIST_X = 2
GO_BACK_DIST = 10
GO_BACK_WAIT = 0.3
LIDAR_SAMPLES = 4
LIDAR_SAMPLES_WAIT = 0.3
MAX_MAP_SIZE = 500
BACK_TO_MAX_CNT = 10
FIX_UP_DIST = 0
WALL_MAX_CNT = 4


class DroneControl:
    def __init__(self, drone_client, final_dest, start_pos):
        self.drone = drone_client
        self.final_dest = final_dest
        self.start_pos = start_pos
        self.current_speed = 0
        self.x_direction, self.y_direction = self.get_direction(final_dest)
        self.last_wall_obs = tuple()
        self.last_xsgin = 1
        self.last_ysgin = 1
        self.last_pos = None
        self.obs_map = np.zeros((2 * MAX_MAP_SIZE, 2 * MAX_MAP_SIZE, 2 * MAX_MAP_SIZE), dtype=np.uint8)
        self.max_speed_cnt = 0
        self.initFinDir()

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
        rot_matrix = self.getRotMatrix(curr_pos)
        global_point = np.matmul(rot_matrix, drone_point) + np.array(
            [curr_pos.pos.x_m, curr_pos.pos.y_m, curr_pos.pos.z_m])
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
        if tuple(curr_dest) == self.final_dest and dist_left < SLOWING_DIST:
            self.current_speed = self.slowdown()
        elif wall:
            self.current_speed = WALL_SPEED
        else:
            self.current_speed = max_possible_speed
        self.current_speed = min(MAX_SPEED, self.current_speed, max_possible_speed)
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

    def goToDest(self, destination, wall=True):
        print("goToDest")
        while not self.reachToDest(destination):
            time.sleep(WALL_WAIT)
            is_obs, _, is_go_up = self.isObsInSight(wall)
            if is_obs:
                break
            elif is_go_up:
                self.drone.flyToPosition(*destination, self.getMaxSpeed(destination))
            elif self.max_speed_cnt >= BACK_TO_MAX_CNT:
                speed = self.getMaxSpeed(destination)
                print_("Increasing to max speed: " + str(speed))
                self.drone.flyToPosition(*destination, speed)
            else:
                self.drone.flyToPosition(*destination, self.current_speed)

    def reachToDest(self, curr_dest):
        print_("reachToDest")
        dist_left = self.getDistToDest(curr_dest)
        print_("dist left")
        print_(dist_left)
        if dist_left <= DEST_ARRIVED_DIST:
            return True
        return False

    def ClearPathToFinalDest(self, no_speed=False, with_turn=False):
        print("ClearPathToFinalDest")
        if no_speed:
            self.current_speed = 0
        if with_turn:
            self.drone.flyToPosition(*self.final_dest, 0)
        is_obs, _, _ = self.isObsInSight()
        if not is_obs:
            self.drone.flyToPosition(*self.final_dest, self.current_speed)

        time.sleep(WAIT_SPIN)
        return (not is_obs), _

    def flyToFinalDest(self):
        print("flyToFinalDest")
        if self.ReachToFinalDest():
            return True
        clear_path, _ = self.ClearPathToFinalDest()
        if clear_path:
            print("There is a Clear path to final dest")
            self.drone.flyToPosition(*self.final_dest, self.getMaxSpeed(self.final_dest))
            self.goToDest(self.final_dest, False)
        else:
            print("Wall detected")
            self.passTheObs()

    def goBack(self, alpha, x, y):
        print("goBack")
        currd = self.getCurrDirect()
        back_vec = (-(math.copysign(1, x) * math.cos(alpha) * GO_BACK_DIST),
                    -(math.copysign(1, y) * GO_BACK_DIST * math.sin(alpha)), 0)
        point = controller.droneP2GlobalP(back_vec, self.drone.getPose())
        speed = self.getMaxSpeed(point, wall=True)
        print_("back vector")
        print_(back_vec)
        self.drone.flyToPosition(*point, speed)
        # self.goToDest(point)
        time.sleep(GO_BACK_WAIT)
        # back_vec = (-GO_BACK_DIST, 0, 0)
        # point = controller.droneP2GlobalP(back_vec, self.drone.getPose())
        # self.drone.flyToPosition(*point, 0)
        self.turnToDest(currd)
        time.sleep(GO_BACK_WAIT)

    def isObsInSight(self, wall=False):
        is_go_up = False
        global MAX_SPEED
        for i in range(LIDAR_SAMPLES):
            print_("isObsInSightLoop")
            time.sleep(LIDAR_SAMPLES_WAIT)
            lid_data = self.drone.getLidarData().points
            if len(lid_data) < 3:
                continue
            for i in range(0, len(lid_data), 3):
                x = lid_data[i]
                y = lid_data[i + 1]
                z = lid_data[i + 2]
                alpha = abs(math.atan(abs(y) / abs(x)))
                self.addToObsMap(x, y, z)
                print_("atan")
                print_(alpha)
                dist = math.sqrt(x ** 2 + y ** 2)
                print_("dist")
                print_(dist)
                if self.obsBenath(x, y, z):
                    self.goUp()
                    is_go_up = True
                if dist < MIN_WALL_DIST_X:
                    print("obs in fly path1")
                    self.goBack(alpha, x, y)
                    self.max_speed_cnt = 0
                    return True, True, is_go_up
                if wall:
                    print_("wall loop")
                    if alpha <= SIGHT_ANGLE_WALL and dist < STOP_THRESH_WALL:
                        print("obs in fly path2")
                        pos = controller.drone.getPose()
                        point = controller.droneP2GlobalP((DEFAULT_FLY_DIST, 0, 0), pos)
                        point[2] = pos.pos.z_m
                        self.drone.flyToPosition(*point, 0)
                        self.current_speed = 0
                        self.max_speed_cnt = 0
                        time.sleep(LIDAR_SAMPLES_WAIT)
                        return True, False, is_go_up
                else:
                    print_("not wall loop")
                    global STOP_THRESH
                    if alpha <= SIGHT_ANGLE and dist < STOP_THRESH:
                        print("obs in fly path3")
                        MAX_SPEED = WALL_SPEED
                        STOP_THRESH = STOP_THRESH_WALL
                        pos = controller.drone.getPose()
                        point = controller.droneP2GlobalP((DEFAULT_FLY_DIST, 0, 0), pos)
                        point[2] = pos.pos.z_m
                        self.drone.flyToPosition(*point, 0)
                        self.current_speed = 0
                        self.max_speed_cnt = 0
                        time.sleep(LIDAR_SAMPLES_WAIT)
                        return True, False, is_go_up
        self.max_speed_cnt = self.max_speed_cnt + 1
        if self.max_speed_cnt >= BACK_TO_MAX_CNT:
            MAX_SPEED = GLOBAL_MAX_SPEED
        return False, False, is_go_up

    def turnRight(self):
        print("turnRight")
        turn_counter = 0
        is_obs, is_back, _ = self.isObsInSight(wall=True)
        while is_obs:
            print("turnRightLoop")
            right_vec = self.getRightVec()
            pos = controller.drone.getPose()
            point = controller.droneP2GlobalP(right_vec, pos)
            point[2] = point[2] - (point[2] - self.final_dest[2]) / 2
            self.drone.flyToPosition(*point, 0)
            time.sleep(WAIT_SPIN)
            turn_counter = turn_counter + 1
            is_obs, is_back, _ = self.isObsInSight(wall=True)
        print_("NUM OF TURNS")
        print_(turn_counter)
        return turn_counter, is_back

    def getRightVec(self):
        # return DEFAULT_FLY_DIST * math.cos(BETA), -DEFAULT_FLY_DIST * math.sin(BETA), 0
        return DEFAULT_FLY_DIST * math.cos(BETA), -DEFAULT_FLY_DIST * math.sin(BETA), FIX_UP_DIST

    def getLeftVec(self, angle):
        return DEFAULT_FLY_DIST * math.cos(angle), DEFAULT_FLY_DIST * math.sin(angle), FIX_UP_DIST

    def turnLeft(self):
        print("turnLeft")
        left_vec = self.getLeftVec(LEFT_ANGLE)
        pos = controller.drone.getPose()
        point = controller.droneP2GlobalP(left_vec, pos)
        point[2] = self.final_dest[2]
        self.drone.flyToPosition(*point, 0)
        time.sleep(WAIT_SPIN)

    def followTheWall(self, is_obs):
        max_loop_cnt = 0
        while is_obs:
            print("followTheWallLoop")
            num_of_turns, is_back = self.turnRight()
            pos = self.drone.getPose()
            glb_point = self.droneP2GlobalP((CLEAR_FLY_DIST, 0, FIX_UP_DIST), pos)
            glb_point[2] = self.final_dest[2]
            speed = self.getMaxSpeed(glb_point, wall=True)
            self.drone.flyToPosition(*glb_point, speed)
            print("curr pos:")
            print(pos)
            print("flying to: ")
            print(glb_point)
            print("with speed: ")
            print(speed)
            self.goToDest(glb_point, True)
            if num_of_turns:
                self.turnLeft()
            is_obs, is_back, _ = self.isObsInSight(wall=True)
            if not is_obs:
                pos = self.drone.getPose()
                glb_point = self.droneP2GlobalP((DEFAULT_FLY_DIST, 0, FIX_UP_DIST), pos)
                # glb_point[2] = pos.pos.z_m
                glb_point[2] = self.final_dest[2]
                speed = self.getMaxSpeed(glb_point)
                self.drone.flyToPosition(*glb_point, speed)
                self.goToDest(glb_point, True)
                is_obs, is_back, _ = self.isObsInSight(wall=True)
            max_loop_cnt = max_loop_cnt + 1
            if max_loop_cnt >= WALL_MAX_CNT:
                print_("Wall cnt exceed follow")
                break
        self.ClearPathToFinalDest(with_turn=True, no_speed=True)

    def calcTriangleArea(self, A, B, C):
        S = (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[0] - B[0])) / 2
        return S

    def isOnVec(self):
        curr_pos = self.drone.getPose()
        pos = np.array([curr_pos.pos.x_m, curr_pos.pos.y_m, curr_pos.pos.z_m])
        if self.calcTriangleArea(self.final_dest, self.start_pos, pos) <= MIN_TRI_AREA:
            print("Drone is on Vec")
            return True
        print("Drone NOT on Vec")
        return False

    def passTheObs(self):
        print("passTheObs")
        self.followTheWall(True)
        while not self.isOnVec():
            self.turnLeft()
            print_("passTheObsLoop")
            self.followTheWall(True)
            self.max_speed_cnt = self.max_speed_cnt + 1
            if self.max_speed_cnt >= WALL_MAX_CNT:
                print_("Wall cnt exceed passTheObs")
                self.max_speed_cnt = 0
                break

    def getCurrDirect(self):
        return self.droneP2GlobalP((DEFAULT_FLY_DIST, 0, 0), self.drone.getPose())

    def turnToDest(self, dir):
        self.drone.flyToPosition(*dir, 0)
        time.sleep(WAIT_SPINL)

    def addToObsMap(self, x, y, z):
        glb_point = self.droneP2GlobalP((x, y, z), self.drone.getPose())
        self.obs_map[(math.floor(glb_point[0]) + MAX_MAP_SIZE) % MAX_MAP_SIZE][
            (math.floor(glb_point[1]) + MAX_MAP_SIZE) % MAX_MAP_SIZE] \
            [(math.floor(glb_point[2]) + MAX_MAP_SIZE) % MAX_MAP_SIZE] = 1

    def obsBenath(self, x, y, z):
        print_("obsBenath")
        if DEBUG:
            print("x: %s y: %s z: %s" % (x, y, z))
        return z == 0

    # return (abs(x) > GO_UP_MIN) and (abs(y) > GO_UP_MIN) and abs(z) < MIN_WALL_DIST_Z

    def goUp(self):
        print("goUp")
        pos = self.drone.getPose()
        glb_point = self.droneP2GlobalP((-1, 0, -5), pos)
        speed = self.getMaxSpeed(glb_point)
        self.drone.flyToPosition(*glb_point, speed)
        time.sleep(WALL_WAIT)

    def initFinDir(self):
        self.drone.flyToPosition(*self.final_dest, 0)
        time.sleep(WAIT_SPIN)


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    start_pos = (-500, -800, -15)  # L shape building
    # dest = (-650, -900, -15) #L shape building
    # start_pos = (-500, -800, -15)
    # dest = (-300, -700, -15) # low building challenge
    dest = (-10, -1000, -15)
    client.setAtPosition(*start_pos)

    time.sleep(3)
    controller = DroneControl(client, dest, start_pos)
    time.sleep(3)
    reach_to_dest = False
    loop_counter = 1

    while True:
        print("Roundddddd: " + str(loop_counter))
        reach_to_dest = controller.flyToFinalDest()
        print("dist to dest: " + str(controller.getDistToFinalDest()))
        if reach_to_dest:
            break

        # time.sleep(wait_time)
        loop_counter = loop_counter + 1
    print("curr dist: " + str(controller.drone.getPose()))

    from matplotlib import pyplot as plt

    plt.rcParams["figure.figsize"] = [7.00, 3.50]
    plt.rcParams["figure.autolayout"] = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    data = controller.obs_map
    z, x, y = data.nonzero()
    ax.scatter(x, y, z, c=z, alpha=1)
    plt.show()
