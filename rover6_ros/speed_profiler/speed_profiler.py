import csv
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt

motors_path = "data/sensors_2020-03-25-19-26-36-rover6-motors.csv"
encoders_path = "data/sensors_2020-03-25-19-26-36-rover6-encoders.csv"
tof_path = "data/sensors_2020-03-25-19-26-36-rover6-tof.csv"

bag_time_format = "%Y/%m/%d/%H:%M:%S.%f"

ticks_per_rotation = 3840.0
wheel_radius_m = 3.25/100.0
m_to_tick_factor = ticks_per_rotation / (2.0 * wheel_radius_m * math.pi)
tick_to_m_factor = 1.0 / m_to_tick_factor

def bag_time_to_date(time_str):
    return datetime.strptime(time_str, bag_time_format)


def time_to_bag_time(timestamp):
    return datetime.strftime(datetime.fromtimestamp(timestamp), bag_time_format)


def load_motor_segments(path):
    with open(path) as file:
        reader = csv.reader(file)

        header = next(reader)
        data = []
        time_index = header.index("time")
        left_index = header.index(".left")
        right_index = header.index(".right")

        for index, line in enumerate(reader):
            bag_time = bag_time_to_date(line[time_index]).timestamp()
            left_cmd = float(line[left_index]) * tick_to_m_factor
            right_cmd = float(line[right_index]) * tick_to_m_factor
            data.append([
                bag_time,
                left_cmd,
                right_cmd,
            ])

    data = np.array(data)

    data_diff = np.diff(data[:, left_index])
    indices = np.where(data_diff != 0.0)[0] + 1

    stop_diff_indices = np.where(data[indices, left_index] == 0.0)[0]
    start_diff_indices = stop_diff_indices + 1
    if start_diff_indices[-1] >= len(indices):
        start_diff_indices[-1] = len(indices) - 1
    stop_indices = indices[stop_diff_indices] - 1
    start_indices = indices[start_diff_indices]
    # start_indices = np.insert(start_indices, 0, 0)

    stop_indices = stop_indices[1:]
    start_indices = start_indices[:-1]

    segments = []
    indices = list(zip(start_indices, stop_indices))
    for start, stop in indices:
        segments.append((
            data[start:stop, time_index],
            data[start:stop, left_index],
            data[start:stop, right_index],
        ))
    return indices, segments, data


def load_encoder_data(path):
    with open(path) as file:
        reader = csv.reader(file)

        header = next(reader)
        data = []
        sec_index = header.index(".header.stamp.secs")
        nsec_index = header.index(".header.stamp.nsecs")
        left_index = header.index(".left_ticks")
        right_index = header.index(".right_ticks")
        left_speed_index = header.index(".left_speed_ticks_per_s")
        right_speed_index = header.index(".right_speed_ticks_per_s")

        for index, line in enumerate(reader):
            timestamp = float(line[sec_index]) + float(line[nsec_index]) * 1E-9
            left = float(line[left_index]) * tick_to_m_factor
            right = float(line[right_index]) * tick_to_m_factor
            left_speed = float(line[left_speed_index]) * tick_to_m_factor
            right_speed = float(line[right_speed_index]) * tick_to_m_factor
            data.append([
                timestamp,
                left,
                right,
                left_speed,
                right_speed,
            ])

    data = np.array(data)
    return data


def load_tof_data(path):
    with open(path) as file:
        reader = csv.reader(file)

        header = next(reader)
        data = []
        sec_index = header.index(".header.stamp.secs")
        nsec_index = header.index(".header.stamp.nsecs")
        front_index = header.index(".front_mm")
        back_index = header.index(".back_mm")

        prev_front = 0.0
        prev_back = 0.0
        prev_time = None
        for index, line in enumerate(reader):
            timestamp = float(line[sec_index]) + float(line[nsec_index]) * 1E-9
            front = float(line[front_index]) / 1000.0
            back = float(line[back_index]) / 1000.0
            if prev_time is None:
                front_speed = 0.0
                back_speed = 0.0
            else:
                front_speed = (front - prev_front) / (timestamp - prev_time)
                back_speed = (back - prev_back) / (timestamp - prev_time)

            prev_front = front
            prev_back = back
            prev_time = timestamp

            data.append([
                timestamp,
                front,
                back,
                front_speed,
                back_speed,
            ])

    data = np.array(data)
    return data


def main():
    indices, segments, motor_data = load_motor_segments(motors_path)
    encoder_data = load_encoder_data(encoders_path)
    # tof_data = load_tof_data(tof_path)

    plt.figure(1)
    plt.plot(motor_data[:, 0], motor_data[:, 1], '-o')
    plt.plot(encoder_data[:, 0], encoder_data[:, 3])
    # plt.plot(tof_data[:, 0], tof_data[:, 3])
    for times, left, right in segments:
        plt.plot(times, left)

    # plt.figure(2)
    # plt.plot(motor_data[:, 0], motor_data[:, 2], '-o')
    # plt.plot(encoder_data[:, 0], encoder_data[:, 4])
    # plt.plot(tof_data[:, 0], tof_data[:, 3])

    plt.show()


if __name__ == '__main__':
    main()
