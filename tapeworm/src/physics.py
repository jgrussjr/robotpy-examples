#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import math
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from pprint import pprint


class PhysicsEngine(object):
    """
        Simulates a motor moving something that strikes two limit switches,
        one on each end of the track. Obviously, this is not particularly
        realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller):
        """
            :param physics_controller: `pyfrc.physics.core.PhysicsInterface` object
                                       to communicate simulation effects to
        """

        self.physics_controller = physics_controller
        self.position = 0

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        # fmt: on

        # setup tape coords
        # Find an object with tape in the name.  Only take the first instance
        tp = self.physics_controller.config_obj['pyfrc']['field']['objects']
        for obj in tp:
            if "tape" in obj['name']:
                point_list = []
                for pt in obj['points']:
                    point_list.append(tuple(pt))
                
                self.tape_box = Polygon(point_list)

                break


    def update_sim(self, hal_data, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.
            
            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        l_motor = hal_data["pwm"][1]["value"]
        r_motor = hal_data["pwm"][2]["value"]

        x, y, angle = self.drivetrain.get_distance(l_motor, r_motor, tm_diff)
        self.physics_controller.distance_drive(x, y, angle)

        # update position (use tm_diff so the rate is constant)
        self.position += hal_data["pwm"][4]["value"] * tm_diff * 3

        # get current pos
        curr_x, curr_y, curr_angle = self.physics_controller.get_position()
        real_current_angle = to_positive_angle(curr_angle)
        #print("Current Angle: " + str(curr_angle))
        #print("Real Angle: " + str(real_current_angle))

        # Calculate the length of the line segments within the robot
        forward_seg_side_mag = self.physics_controller.config_obj['pyfrc']['robot']['w'] / 2
        wid_seg_side_mag = self.physics_controller.config_obj['pyfrc']['robot']['h'] / 2

        # find the end point of the line representing the forward pointing side of the right triange 
        forward_end_x = curr_x + (forward_seg_side_mag * math.cos(real_current_angle))
        forward_end_y = curr_y + (forward_seg_side_mag * math.sin(real_current_angle))

        # point representing the the 'front middle' sensor
        sensor_front_cntr_pos = Point(forward_end_x, forward_end_y)
        # slope of the forward line
        forward_line_slope = (forward_end_y - curr_y) / (forward_end_x - curr_x)

        # find the two other sensor points, forming a perpendicular line to our forward line
        t_a, t_b = findPointsOnLine((forward_end_x, forward_end_y), wid_seg_side_mag, (-1/ (forward_line_slope + .0000000001) ))

        # We have two points, need to decide on which the front 'right' vs 'left'
        # they are using a clockwise circle for some reason
        if real_current_angle > math.pi and real_current_angle <= math.pi * 2:
            # we are pointing into the 1,2 quadrant
            if t_a.coords[0] > sensor_front_cntr_pos.coords[0]:
                # this point is our 'right' point
                sensor_front_side_r_pos = t_a
                sensor_front_side_l_pos = t_b
                #print("1-2: A")
            elif t_b.coords[0] > sensor_front_cntr_pos.coords[0]:
                # this point is our 'right' point
                sensor_front_side_r_pos = t_b
                sensor_front_side_l_pos = t_a
                #print("1-2: B")
        elif real_current_angle < math.pi: 
            # we are in quadrant 3,4
            if t_a.coords[0] < sensor_front_cntr_pos.coords[0]:
                # this point is our 'right' point
                sensor_front_side_r_pos = t_a
                sensor_front_side_l_pos = t_b
                #print("3-4: A")
            elif t_b.coords[0] < sensor_front_cntr_pos.coords[0]:
                # this point is our 'right' point
                sensor_front_side_r_pos = t_b
                sensor_front_side_l_pos = t_a
                #print("3-4: B")
        else:
            print("ERROR: angle comparison is beyond 2*pi")

        # Check sensor values
        sensor_front_cntr = self.tape_box.contains(sensor_front_cntr_pos)
        sensor_front_side_l = self.tape_box.contains(sensor_front_side_l_pos)
        sensor_front_side_r = self.tape_box.contains(sensor_front_side_r_pos)

        # set values here
        hal_data["dio"][4]["value"] = sensor_front_cntr
        hal_data["dio"][5]["value"] = sensor_front_side_l
        hal_data["dio"][6]["value"] = sensor_front_side_r

        hal_data["analog_in"][2]["voltage"] = self.position


def findPointsOnLine(src_pnt, length,  slope):
    a = [None, None]
    b = [None, None]

    if slope is 0: 
        a[0] = src_pnt[0] + length
        a[1] = src_pnt[1]

        b[0] = src_pnt[0] - length
        b[1] = src_pnt[1]

    # if slope is infinte 
    elif slope is float("inf"):
        a[0] = src_pnt[0]
        a[1] = src_pnt[1] + length

        b[0] = src_pnt[0]
        b[1] = src_pnt[1] - length
    else:
        dx = (length / math.sqrt(1 + (slope * slope)))
        dy = slope * dx
        a[0] = src_pnt[0] + dx
        a[1] = src_pnt[1] + dy
        b[0] = src_pnt[0] - dx
        b[1] = src_pnt[1] - dy

        return Point(a[0], a[1]), Point(b[0], b[1])

def to_positive_angle(angle_rad):
    angle_rad = math.fmod(angle_rad, 2*math.pi)
    if angle_rad < 0:
        angle_rad += 2*math.pi
    return angle_rad