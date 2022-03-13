import numpy as np

from DroneClient import DroneClient
import time
import math
import airsim.utils

DEST_ARRIVED_DIST = 2
SLOWING_DIST = 7
MAX_SPEED = 5
MAX_TIME = 0.62
URBAN_SPEED = 5
WALL_SPEED = 5
WALL_WAIT = 1
WAIT_LIDAR = 0.1
STOP_THRESH = 10
WAIT_SPIN = 2
BETA = 1.4  # rad
DEFAULT_FLY_WAIT = 0.5
MIN_WALL_DIST = 3
GO_BACK_DIST = 15


class DroneControl:
    def __init__(self, client, final_dest):
        self.drone = client
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

    def droneP2GlobalP(self, data, pos):
        print("droneP2GlobalP")
        drone_point = np.array([data.points[0], 0, 0])
        print("pos")
        print(pos)
        print("drone_point")
        print(drone_point)
        rot_matrix = self.getRotMatrix(pos)
        global_point = np.matmul(rot_matrix, drone_point) + np.array([pos.pos.x_m, pos.pos.y_m, pos.pos.z_m])
        print("R*P")
        print(np.matmul(rot_matrix, drone_point))
        print("global_point")
        print(global_point)
        #   exit()
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

    def getMaxSpeed(self, dest, wall=False):
        dist_left = self.getDistToFinalDest()
        max_possible_speed = dist_left / MAX_TIME
        if dest == self.final_dest and dist_left < SLOWING_DIST:
            self.current_speed = self.slowdown()
        elif wall:
            self.current_speed = WALL_SPEED
        else:
            self.current_speed = max_possible_speed
        self.current_speed = min(MAX_SPEED, self.current_speed, max_possible_speed)
        return self.current_speed

    def ReachToFinalDest(self):
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

    def reachToDest(self, dest):
        dist_left = self.getDistToDest(dest)
        if dist_left <= DEST_ARRIVED_DIST:
            return True
        return False

    def checkWallInFront(self):
        lid_data = self.drone.getLidarData()
        print("check_clear_path, lidar pointsss")
        print(lid_data.points)
        if len(lid_data.points) >= 3 and STOP_THRESH > lid_data.points[0]:
            self.current_speed = 0
            self.drone.flyToPosition(*self.final_dest, self.current_speed)
            return True
        return False

    def ClearPathToFinalDest(self, no_speed=False):
        if no_speed:
            self.current_speed = 0
        self.drone.flyToPosition(*self.final_dest, self.current_speed)
        time.sleep(WAIT_SPIN)
        if self.checkWallInFront():
            return False
        time.sleep(WAIT_LIDAR)
        if self.checkWallInFront():
            return False
        return True

    def flyToFinalDest(self):
        if self.ReachToFinalDest():
            return True

        elif self.ClearPathToFinalDest():
            print("Clear path to final dest")
            self.drone.flyToPosition(*self.final_dest, self.getMaxSpeed(self.final_dest))
            time.sleep(DEFAULT_FLY_WAIT)

        else:
            print("Wall detected")
            self.followTheWall()

    def turnToFinalDest(self):
        self.current_speed = 0
        self.drone.flyToPosition(*self.final_dest, self.current_speed)
        time.sleep(WAIT_SPIN)

    def turnToLastObs(self):
        print("turnToLastObs")
        pos = self.drone.getPose()
        dist = math.sqrt((pos.pos.x_m - self.last_pos.pos.x_m) ** 2 + (pos.pos.y_m - self.last_pos.pos.y_m) ** 2)
        z = math.tan(BETA)*dist
        xd = math.sin(BETA) * z
        yd = math.cos(BETA) * z
        zd = self.final_dest[2]
        new_point = (pos.pos.x_m + self.last_xsgin * xd, pos.pos.y_m - self.last_ysgin * yd, zd)
        self.current_speed = 0
        self.drone.flyToPosition(*new_point, self.current_speed)
        print(new_point)
        time.sleep(WAIT_SPIN)

    def flyToLastObs(self):
        print("flyToLastObs")
        pos = self.drone.getPose()
        dist = math.sqrt((pos.pos.x_m - self.last_pos.pos.x_m) ** 2 + (pos.pos.y_m - self.last_pos.pos.y_m) ** 2)
        z = math.tan(BETA)*dist
        xd = math.sin(BETA) * z
        yd = math.cos(BETA) * z
        zd = self.final_dest[2]
        new_point = (pos.pos.x_m + self.last_xsgin * xd, pos.pos.y_m - self.last_ysgin * yd, zd)
        self.current_speed = WALL_SPEED
        self.drone.flyToPosition(*new_point, self.current_speed)
        print(new_point)
        time.sleep(WAIT_SPIN)

    def getShiftDist(self, pos, obs_point):
        #   radius = float(math.sqrt((pos.pos.x_m-obs_point[0]) ** 2 + (pos.pos.y_m-obs_point[1]) ** 2) )/ 2
        radius = 25
        xd = math.cos(BETA) * radius
        yd = math.sin(BETA) * radius
        return xd, yd

    def goBack(self, pos, obs):
        print("goBack")
        xdir = obs[0] - pos.pos.x_m
        ydir = obs[1] - pos.pos.y_m
        obs_list = list(obs)
        self.last_wall_obs = tuple(obs_list)
        obs_list[0] = obs[0] - xdir * GO_BACK_DIST
        obs_list[1] = obs[1] - ydir * GO_BACK_DIST
        obs_list[2] = self.final_dest[2]
        return tuple(obs_list)

    def getNewDest(self, pos, obs_point):
        print("getNewDest")
        dist = math.sqrt((pos.pos.x_m - obs_point[0]) ** 2 + (pos.pos.y_m - obs_point[1]) ** 2)
        if dist < MIN_WALL_DIST:
            # pass
            return self.goBack(pos, obs_point)

        xsign = 1
        ysign = -1
        if pos.pos.x_m > obs_point[0]:
            xsign = -1
        if pos.pos.y_m > obs_point[1]:
            ysign = 1
        xd, yd = self.getShiftDist(pos, obs_point)
        zd = self.final_dest[2]
        new_point = (pos.pos.x_m + xsign * xd, pos.pos.y_m + ysign * yd, zd)
        obs_list = list(obs_point)
        obs_list[0] = obs_list[0] + xsign * xd
        obs_list[1] = obs_list[1] + ysign * yd
        self.last_wall_obs = tuple(obs_list)
        self.last_xsgin = xsign
        self.last_ysgin = ysign
        self.last_pos = pos
        return new_point

    def followTheWall(self):
        self.turnToFinalDest()
        lid_data = self.drone.getLidarData()
        while len(lid_data.points) >= 3 and STOP_THRESH > lid_data.points[0]:
            print("followTheWall")
            print("lid:")
            print(lid_data.points)
            pos = self.drone.getPose()
            obs_point = self.droneP2GlobalP(lid_data, pos)
            new_point = self.getNewDest(pos, obs_point)
            speed = self.getMaxSpeed(new_point, wall=True)
            self.drone.flyToPosition(*new_point, speed)
            print("curr pos:")
            print(pos)
            print("curr obs_point:")
            print(obs_point)
            print("flying to: ")
            print(new_point)
            print("with speed: ")
            print(speed)
            while not self.reachToDest(new_point):
                time.sleep(WAIT_SPIN)
                lid_data = self.drone.getLidarData()
                if lid_data.points[0] < MIN_WALL_DIST:
                    break
            self.turnToLastObs()
            lid_data = self.drone.getLidarData()
            pos = self.drone.getPose()
            print("curr pos:")
            print(pos)
            print("lid_data:")
            print(lid_data)
            #  exit()
            if len(lid_data.points) < 3 or STOP_THRESH > lid_data.points[0]:
                time.sleep(WAIT_LIDAR)
                lid_data = self.drone.getLidarData()
                if len(lid_data.points) < 3 or STOP_THRESH < lid_data.points[0]:
                    self.flyToLastObs()


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-300, -700, -20)

    time.sleep(3)
    dest = (-400, -800, -20)
    controller = DroneControl(client, dest)
    time.sleep(0.5)
    reach_to_dest = False
    loop_counter = 0

    while True:
        print("Roundddddd: " + str(loop_counter))
        reach_to_dest = controller.flyToFinalDest()
        if reach_to_dest:
            break

        # time.sleep(wait_time)
        loop_counter = loop_counter + 1
