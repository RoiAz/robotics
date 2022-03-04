from DroneClient import DroneClient
import time
import math
import airsim.utils

pwr_factor = 2
fix_factor = -10
coeff_factor = 35
wait_time = 1
MAX_TIME = 0.62
slowing_distance = 3
DEST_ARRIVED_DIST = 3
MAX_SPEED = 10
WALL_SPEED = 5
WALL_DELTA = 10
WALL_WAIT = 1
STOP_THRESH = 35
BACK_DIST = 10
MAX_LIDAR = 35

class DroneControl:
    def __init__(self, client, final_dest):
        self.drone = client
        self.curr_flight_dest = final_dest
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

    def get_dist(self, dest, with_z=False):
        curr_pos = self.drone.getPose()
        curr_x = curr_pos.pos.x_m
        curr_y = curr_pos.pos.y_m
        dest_x = dest[0]
        dest_y = dest[1]
        if with_z:
            curr_z = curr_pos.pos.z_m
            dest_z = dest[2]
            dist_left = math.sqrt((dest_x - curr_x) ** 2 + (dest_y - curr_y) ** 2 + (dest_z - curr_z) ** 2)
        else:
            dist_left = math.sqrt((dest_x - curr_x) ** 2 + (dest_y - curr_y) ** 2)
        return dist_left

    def create_new_dest(self):
        dest_x = self.curr_flight_dest[0]
        dest_y = self.curr_flight_dest[1]
        dest_z = self.curr_flight_dest[2]
        lid_data = self.drone.getLidarData()
        print("lidar pointsss")
        print(lid_data.points[0])
        #  x_delta = math.copysign(1, dest_x) * coeff_factor * math.exp(pwr_factor * lid_data.points[0])
        x_delta = math.copysign(1, dest_x) * coeff_factor * lid_data.points[0]
        print("x delta:")
        print(x_delta)
        new_dest_x = dest_x + x_delta
        # y_delta = math.copysign(1, dest_y) * coeff_factor * math.exp(pwr_factor * lid_data.points[0])
        y_delta = math.copysign(1, dest_y) * coeff_factor * lid_data.points[0]
        new_dest_y = dest_y - y_delta
        curr_pos = self.drone.getPose()
        curr_z = curr_pos.pos.z_m
        curr_x_rad = curr_pos.orientation.x_rad
        new_dest_z = dest_z + math.exp(curr_x_rad * fix_factor) * (dest_z - curr_z) * math.log(
            max(2, self.current_speed))
        dest_lst = list(self.curr_flight_dest)
        dest_lst[0] = new_dest_x
        dest_lst[1] = new_dest_y
        dest_lst[2] = new_dest_z
        self.curr_flight_dest = tuple(dest_lst)
        return self.curr_flight_dest

    def isPassedTheWall(self, dest, last_dist_from_wall):
        self.drone.flyToPosition(*self.final_dest, 0)
        #time.sleep(wait_time)
        lid_data = self.drone.getLidarData()
        print("lidar pointsss")
        print(lid_data)
        curr_dist = lid_data.points[0]
        print("isPassedTheWall:")
        print(last_dist_from_wall)
        print(curr_dist)
        print("$" * 10)
        if curr_dist > last_dist_from_wall or len(lid_data.points) < 3:
            return True
        return False

    def createWallDest(self, dest):
        curr_pos = self.drone.getPose()
        curr_x = curr_pos.pos.x_m
        curr_y = curr_pos.pos.y_m
        curr_z = curr_pos.pos.z_m
        dest_lst = list(dest)
        dest_lst[0] = curr_x - self.x_direction * BACK_DIST
        dest_lst[1] = curr_y + self.y_direction * WALL_DELTA
        dest_lst[2] = curr_z
        self.curr_flight_dest = tuple(dest_lst)
        print("createWallDest:")
        print(curr_pos)
        print(self.curr_flight_dest)
        print("%" * 10)
        return self.curr_flight_dest

    def followTheWall(self, dest):
        print("followTheWall:")
        lid_data = self.drone.getLidarData()
        last_dist_from_wall = lid_data.points[0]
        print("lidar pointsss")
        print(lid_data)
        while not self.isPassedTheWall(dest, last_dist_from_wall):
            print("followTheWall:")
            new_dest = self.createWallDest(dest)
            self.drone.flyToPosition(*new_dest, WALL_SPEED)
            time.sleep(WALL_WAIT)

    def fly_to_dest(self, dest):
        """
        Fly the drone to position
        Args:
            x : float - x coordinate
            y : float - y coordinate
            z : float - z coordinate
            v : float - the velocity which the drone fly to position
        """
        print("current pose:")
        print(client.getPose())
        if self.check_reach_to_final_dest():
            # stop speed
            return True

        if self.check_clear_path(dest):
            dest_z = self.final_dest[2]
            curr_pos = self.drone.getPose()
            curr_z = curr_pos.pos.z_m
            # curr_x_rad = curr_pos.orientation.x_rad
            curr_x_rad = 1
            new_dest_z = dest_z + math.copysign(1, dest_z) * math.exp(curr_x_rad * fix_factor) * (
                    dest_z - curr_z) * math.log(max(2, self.current_speed))
            dest_lst = list(dest)
            dest_lst[2] = new_dest_z
            self.curr_flight_dest = tuple(dest_lst)
            speed = self.get_max_speed(self.curr_flight_dest)
            # curr_x_rad = curr_pos.orientation.x_rad
            curr_x_rad = 1
            new_dest_z = dest_z + math.copysign(1, dest_z) * math.exp(curr_x_rad * fix_factor) * (
                    dest_z - curr_z) * math.log(max(2, self.current_speed))
            dest_lst = list(dest)
            dest_lst[2] = new_dest_z
            self.curr_flight_dest = tuple(dest_lst)
            print("when clear fly to:")
            print(self.curr_flight_dest)
            print("with speed:")
            print(speed)
            self.drone.flyToPosition(*self.curr_flight_dest, speed)
            time.sleep(wait_time)

        else:
            self.followTheWall(dest)

            # new_dest = dest
            # while not self.check_clear_path(new_dest):
            #     print("when wall fly to:")
            #     new_dest = self.create_new_dest()
            #     print(new_dest)
            #     speed = self.get_max_speed(new_dest, obs=True)
            #     print("with speed:")
            #     print(speed)
            #     self.drone.flyToPosition(*new_dest, speed)
            #     print("current pose:")
            #     print(client.getPose())
            #     time.sleep(wait_time)

            # follow until no wall, create new dest with sensitivity parameter to change the angle
            # if no wall move break and move to dest
            # need to check also that we don't move for nothing, false alarm if dest is close than obstacle
        return False

    def slowdown(self, obs=False):
        if obs:
            lid_data = self.drone.getLidarData()
            speed = math.exp(pwr_factor * lid_data.points[0])
        else:
            speed = int(self.current_speed / 2)
        return speed

    def get_max_speed(self, dest, obs=False):
        dist_left = self.get_dist(dest)
        if (dest == self.final_dest and dist_left < slowing_distance):
            self.current_speed = self.slowdown()
        # elif if we have obstcale fly slower ?
        elif obs:
            self.current_speed = self.slowdown(obs=True)
        else:
            max_speed = dist_left / MAX_TIME
            self.current_speed = max_speed
        self.current_speed = min(MAX_SPEED, self.current_speed)
        return self.current_speed

    def check_reach_to_final_dest(self):
        dist_left = self.get_dist(self.final_dest, with_z=False)
        if self.check_clear_path(self.final_dest) and dist_left <= DEST_ARRIVED_DIST:
            return True
        return False

    def check_clear_path(self, dest):
        lid_data = self.drone.getLidarData()
        print("check_clear_path, lidar pointsss")
        print(lid_data.points[0])
        if len(lid_data.points) < 3:
            return True
        dist_left = self.get_dist(dest)
        if dist_left < lid_data.points[0]:
            return True
        elif STOP_THRESH < lid_data.points[0]:
            return True
        self.drone.flyToPosition(*self.final_dest, 0)
        #curr_pos = self.drone.getPose()
        # curr_x = curr_pos.pos.x_m
        # curr_y = curr_pos.pos.y_m
        # curr_z = curr_pos.pos.z_m
        # # dest_lst = list(dest)
        # # back_dist = (MAX_LIDAR - lid_data.points[0]) / 1.4
        # # dest_lst[0] = curr_x - self.x_direction * back_dist
        # # dest_lst[1] = curr_y - self.y_direction * back_dist
        # # dest_lst[2] = curr_z
        # # coll_dest = tuple(dest_lst)
        # # self.drone.flyToPosition(*coll_dest, WALL_SPEED)
        # # time.sleep(wait_time)
        # # dest_lst = list(dest)
        # # dest_lst[0] = curr_x
        # # dest_lst[1] = curr_y
        # # dest_lst[2] = curr_z
        # # coll_dest = tuple(dest_lst)
        # # self.drone.flyToPosition(*coll_dest, 0)
        # # time.sleep(wait_time)
        return False


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -50)

    time.sleep(3)
    dest = (-200, -800, -50)
    controller = DroneControl(client, dest)
    reach_to_dest = False
    loop_counter = 0

    while True:
        print("Roundddddd: " + str(loop_counter))
        # if lidar smaller than threshold make a move
        # clear_path_to_dest = controller.check_clear_path(dest)

        reach_to_dest = controller.fly_to_dest(dest=dest)
        if reach_to_dest:
            break

        time.sleep(wait_time)
        loop_counter = loop_counter + 1
