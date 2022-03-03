from DroneClient import DroneClient
import time
import math
import numpy
import airsim.utils

pwr_factor = -2
coeff_factor = 35
wait_time = 1
MAX_TIME = 0.62
slowing_distance = 3
DEST_ARRIVED_DIST = 1.5


class DroneControl:
    def __init__(self, client, final_dest):
        self.drone = client
        self.curr_flight_dest = final_dest
        self.final_dest = final_dest
        self.current_speed = 0
        self.fly_to_dest(final_dest)

    def get_dist(self, dest):
        curr_pos = self.drone.getPose()
        curr_x = curr_pos.pos.x_m
        curr_y = curr_pos.pos.y_m
        dest_x = dest[0]
        dest_y = dest[1]
        dist_left = math.sqrt((dest_x - curr_x) ** 2 + (dest_y - curr_y) ** 2)
        return dist_left

    def create_new_dest(self):
        dest_x = self.curr_flight_dest[0]
        lid_data = self.drone.getLidarData()
        points = self.parse_lidarData(lid_data)
        new_dest_x = dest_x + coeff_factor * math.exp(pwr_factor * points[0])
        self.curr_flight_dest[0] = new_dest_x
        return self.curr_flight_dest

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
            self.curr_flight_dest = dest
            speed = self.get_max_speed(dest)
            print("when clear fly to:")
            print(dest)
            self.drone.flyToPosition(*dest, speed)
        else:
            while not self.check_clear_path(dest):
                new_dest = self.create_new_dest()
                speed = self.get_max_speed(new_dest)
                print("when wall fly to:")
                print(new_dest)
                self.drone.flyToPosition(*dest, speed)
                time.sleep(wait_time)

            # follow until no wall, create new dest with sensitivity parameter to change the angle
            # if no wall move break and move to dest
            # need to check also that we don't move for nothing, false alarm if dest is close than obstacle
        return False

    def slowdown(self):
        return int(self.current_speed / 2)

    def get_max_speed(self, dest):
        dist_left = self.get_dist(dest)
        if dest == self.final_dest and dist_left < slowing_distance:
            self.current_speed = self.slowdown()
        # elif if we have obstcale fly slower ?
        else:
            max_speed = dist_left / MAX_TIME
            print("dist_left: " + str(dist_left))
            self.current_speed = max_speed
        print("speed: " + str(self.current_speed))
        return self.current_speed

    def check_reach_to_final_dest(self):
        dist_left = self.get_dist(self.final_dest)
        if self.check_clear_path(self.final_dest) and dist_left <= DEST_ARRIVED_DIST:
            return True
        return False

    def parse_lidarData(self, data):
        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0] / 3), 3))
        return points

    def check_clear_path(self, dest):
        lid_data = self.drone.getLidarData()
        print(lid_data.points)
        if len(lid_data.points) < 3:
            return True
        dist_left = self.get_dist(dest)
        if dist_left < lid_data.points[0]:
            return True
        return False


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    time.sleep(3)
    dest = (-346, -420, -100)
    controller = DroneControl(client, dest)
    reach_to_dest = False
    loop_counter = 0

    while True:
        print ("Roundddddd: " + str(loop_counter))
        # if lidar smaller than threshold make a move
        # clear_path_to_dest = controller.check_clear_path(dest)

        reach_to_dest = controller.fly_to_dest(dest=dest)
        if reach_to_dest:
            break

        print(client.getLidarData())

        time.sleep(wait_time)
        loop_counter = loop_counter + 1
