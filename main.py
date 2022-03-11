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
WAIT_LIDAR = 0.1
STOP_THRESH = 12
WAIT_SPIN = 0.5

DEFAULT_FLY_WAIT = 1


class DroneControl:
    def __init__(self, client, final_dest):
        self.drone = client
        self.final_dest = final_dest
        self.current_speed = 0

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
        ry = self.calcRx(pos.orientation.y_rad)
        rz = self.calcRx(pos.orientation.z_rad)
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
        drone_point = np.array([data.points[0], data.points[1], data.points[2]])
        rot_matrix = self.getRotMatrix(pos)
        global_point = np.matmul(rot_matrix, drone_point) + np.array([pos.pos.x_m, pos.pos.y_m, pos.pos.z_m])
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
        print(lid_data.points[0])
        if len(lid_data.points) > 3 and STOP_THRESH > lid_data.points[0]:
            self.current_speed = 0
            self.drone.flyToPosition(*self.final_dest, self.current_speed)
            return True
        return False

    def ClearPathToFinalDest(self):
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
            self.drone.flyToPosition(*self.final_dest, self.getMaxSpeed())
            time.sleep(DEFAULT_FLY_WAIT)

        else:
            self.followTheWall()

    def turnToFinalDest(self):
        self.current_speed = 0
        self.drone.flyToPosition(*self.final_dest, self.current_speed)
        time.sleep(WAIT_SPIN)

    def getNewDest(self, pos, glb_point):
        if pos.pos.x_m > glb_point[0]:
            xsign = -1

    def followTheWall(self):
        self.turnToFinalDest()
        lid_data = self.drone.getLidarData()
        while len(lid_data.points) > 3 and STOP_THRESH > lid_data.points[0]:
            pos = self.drone.getPose()
            glb_point = self.droneP2GlobalP(lid_data, pos)


            self.turnToFinalDest()
            lid_data = self.drone.getLidarData()



if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(0, 0, -20)

    time.sleep(3)
    dest = (0, -800, -20)
    controller = DroneControl(client, dest)
    reach_to_dest = False
    loop_counter = 0

    while True:
        print("Roundddddd: " + str(loop_counter))
        reach_to_dest = controller.flyToFinalDest()
        if reach_to_dest:
            break

        # time.sleep(wait_time)
        loop_counter = loop_counter + 1
