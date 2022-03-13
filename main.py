import numpy as np

from DroneClient import DroneClient
import time
import math
import airsim.utils
import numpy

DEST_ARRIVED_DIST = 3
SLOWING_DIST = 7
MAX_SPEED = 15
MAX_TIME = 0.62
URBAN_SPEED = 5
WALL_SPEED = 5
WALL_WAIT = 0.5
WAIT_LIDAR = 0.1
STOP_THRESH = 20
WAIT_SPIN = 1
BETA = 0.8  # rad
DEFAULT_FLY_WAIT = 1
MIN_WALL_DIST = 3
GO_BACK_DIST = 10

class DroneControl:
    def __init__(self, client, final_dest):
        self.drone = client
        self.final_dest = final_dest
        self.current_speed = 0
        self.x_direction, self.y_direction = self.get_direction(final_dest)

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
        drone_point = np.array([data.points[0], data.points[1], data.points[2]])
        print(drone_point)
        rot_matrix = self.getRotMatrix(pos)
        print(rot_matrix)
        #  print(np.matmul(rot_matrix, drone_point))
        global_point = np.matmul(rot_matrix, drone_point) + np.array([pos.pos.x_m, pos.pos.y_m, pos.pos.z_m])
      #  global_point = np.matmul(drone_point, rot_matrix) + np.array([pos.pos.x_m, pos.pos.y_m, pos.pos.z_m])
        print(data)
        print("-"*10)
        print(np.matmul(drone_point, rot_matrix))
        print("-" * 10)
        print(pos)
        #  print("-" * 10)
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

    def getShiftDist(self, pos, obs_point):
     #   radius = float(math.sqrt((pos.pos.x_m-obs_point[0]) ** 2 + (pos.pos.y_m-obs_point[1]) ** 2) )/ 2
        radius = 20
        xd = math.cos(BETA) * radius
        yd = math.sin(BETA) * radius
        return xd, yd

    def goBack(self, pos, obs):
        print("goBack")
        xdir = obs[0] - pos.pos.x_m
        ydir = obs[1] - pos.pos.y_m
        return (obs[0] - xdir * GO_BACK_DIST, obs[1] - ydir * GO_BACK_DIST, self.final_dest[2])

    def getNewDest(self, pos, obs_point):
        print("getNewDest")
        dist = math.sqrt((pos.pos.x_m - obs_point[0]) ** 2 + (pos.pos.y_m - obs_point[1]) ** 2)
        if dist < MIN_WALL_DIST:
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
        return new_point

    def followTheWall(self):
        print("followTheWall")
        self.turnToFinalDest()
        lid_data = self.drone.getLidarData()
        print(lid_data.points)
        while len(lid_data.points) >= 3 and STOP_THRESH > lid_data.points[0]:
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
            time.sleep(WAIT_SPIN)
            self.turnToFinalDest()
            lid_data = self.drone.getLidarData()
            if len(lid_data.points) < 3:
                time.sleep(WALL_WAIT)
                lid_data = self.drone.getLidarData()


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
