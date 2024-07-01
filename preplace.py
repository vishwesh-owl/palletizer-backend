import socket
import struct
import time
import math

DEFAULT_TIMEOUT = 10

# Connection and configuration details omitted for brevity

class RobotData():
    # Robot connection and data handling methods omitted for brevity

def get_tcp_pose(robot_ip):
    # TCP pose retrieval method omitted for brevity

def getMasterPoint():
    robot_ip = '192.168.1.200'
    input("Press enter after reaching desired master location and switch robot to Remote mode")
    result = get_tcp_pose(robot_ip)
    print(result)

    input("Press enter after reaching desired pickup location")
    pickup = get_tcp_pose(robot_ip)
    print(pickup)

    num_layers = int(input("Enter the number of layers: "))

    return pickup, result, num_layers

def calculate_pre_point(point):
    return [point[0], point[1], point[2] + 0.2, point[3], point[4], point[5]]

def apply_rotation(position, angle_rad):
    position[5] += angle_rad
    return position

def calculate_pre_placement_point(box_coords, largest_dimension, current_box_index):
    x_offset = 0.1  # Offset value for x coordinate
    y_offset = 0.1  # Offset value for y coordinate
    z_offset = 0.2  # Constant z offset
    current_box = box_coords[current_box_index]
    x_signs = []
    y_signs = []

    for i in range(current_box_index):
        previous_box = box_coords[i]
        x_diff = current_box[0] - previous_box[0]
        y_diff = current_box[1] - previous_box[1]

        if -largest_dimension <= x_diff <= largest_dimension:
            if x_diff > 0:
                x_signs.append(1)
            elif x_diff < 0:
                x_signs.append(-1)

        if -largest_dimension <= y_diff <= largest_dimension:
            if y_diff > 0:
                y_signs.append(1)
            elif y_diff < 0:
                y_signs.append(-1)

    pre_place_x = current_box[0] + (x_offset if x_signs.count(1) > x_signs.count(-1) else -x_offset if x_signs.count(-1) > x_signs.count(1) else 0)
    pre_place_y = current_box[1] + (y_offset if y_signs.count(1) > y_signs.count(-1) else -y_offset if y_signs.count(-1) > y_signs.count(1) else 0)
    pre_place_z = master_point[2] + z_offset

    return [pre_place_x, pre_place_y, pre_place_z, 0, 0, 0]  # Ensure the list has six elements

if __name__ == "__main__":
    pickup_point, master_point, num_layers = getMasterPoint()

    # Define box coordinates and largest dimension
    largest_dimension = 3
    box_coords = [
        [1.5, 1, 0],  # b1
        [4.5, 1, 0],  # b2
        [1, 3.5, 0],  # b3
        [3, 3.5, 0],  # b4
        [5, 3.5, 0]   # b5
    ]

    rb = RobotData()
    try:
        rb.connect('192.168.1.200')

        for layer in range(num_layers):
            for i, box in enumerate(box_coords):
                pre_pickup = calculate_pre_point(pickup_point)
                pre_place = calculate_pre_placement_point(box_coords, largest_dimension, i)

                rb.movel(pre_pickup)
                time.sleep(3)
                rb.movel(pickup_point)
                time.sleep(2)
                rb.movel(pre_pickup)
                time.sleep(2)

                # Apply rotation to pre-place, place, and pre-place (second time)
                rotation_angle = box[2]  # Get the rotation angle
                pre_place_rotated = apply_rotation(pre_place.copy(), rotation_angle)
                box_abs_rotated = apply_rotation([box[0], box[1], master_point[2], 0, 0, 0], rotation_angle)  # Ensure correct z offset

                rb.movel(pre_place_rotated)
                time.sleep(3)
                rb.movel(box_abs_rotated)
                time.sleep(3)
                rb.movel(pre_place_rotated)
                time.sleep(2)

                # Apply rotation to pre-pickup for next pickup
                pre_pickup_rotated = apply_rotation(pre_pickup.copy(), -rotation_angle)
                rb.movel(pre_pickup_rotated)
                time.sleep(3)

            # Adjust height for next layer
            pickup_point[2] += 0
            master_point[2] += 0.1

    except (socket.error, socket.timeout) as e:
        print(f"Socket error: {e}")
    finally:
        rb.disconnect()

    print("Pickup Point:", pickup_point)
    print("Master Point:", master_point)
    print("Number of Layers:", num_layers)
