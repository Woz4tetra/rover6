import argparse
import datetime
import numpy as np
# import pandas as pd
import dateutil.parser
import matplotlib.pyplot as plt

try:
    from lib.config import ConfigManager
    from lib.nodes.data_logger import DataLogger

    data_log_config = ConfigManager.get_data_log_config()
except BaseException as e:
    print("Ignoring configs")
    print(e)
    data_log_config = None
    ConfigManager = None
    DataLogger = None


parser = argparse.ArgumentParser(description="Parse Data Logger output")
parser.add_argument("path", type=str, nargs="?", help="Path to data file")
parser.add_argument("--start_date", "-s", type=str, nargs="?", help="Start of date range")
parser.add_argument("--stop_date", "-x", type=str, nargs="?", help="Stop of date range")
args = parser.parse_args()

def main():
    if args.path is not None:
        path = args.path
    elif data_log_config is not None:
        path = data_log_config.path
    else:
        raise ValueError("Config paths not set and path argument not provided!")

    if DataLogger is not None:
        start_flag = DataLogger.start_flag
    else:
        start_flag = "---- Data Logger start -----"

    if args.start_date is not None:
        start_date = dateutil.parser.parse(args.start_date)
    else:
        start_date = None

    if args.stop_date is not None:
        stop_date = dateutil.parser.parse(args.stop_date)
    else:
        stop_date = None

    with open(path) as file:
        contents = file.read()

    ina_data = {
        "time"      : [],
        "voltage_V" : [],
        "current_mA": [],
    }
    cpu_temp_data = {
        "time"   : [],
        "celsius": [],
    }

    raw_data = []
    for line in contents.splitlines():
        line = line.strip()
        if len(line) == 0:
            continue
        date_str, msg = line.split(":\t", 1)
        date = datetime.datetime.strptime(date_str, "%Y-%m-%dT%H:%M:%S,%f")
        if msg == start_flag:
            print(line)
            continue

        if start_date is not None:
            if date < start_date:
                continue
        if stop_date is not None:
            if date > stop_date:
                continue

        raw_data.append((date, msg))

    for date, msg in raw_data:
        line_data = msg.split("\t")
        identifier = line_data[0]
        if identifier == "ina":
            try:
                timestamp, voltage_V, current_mA = list(map(float, line_data[1:]))
            except ValueError as e:
                print(e, msg)
                continue
            ina_data["time"].append(timestamp)
            ina_data["voltage_V"].append(voltage_V)
            ina_data["current_mA"].append(current_mA)
        elif identifier == "camera_detected":
            if line_data[1] != "supported=1 detected=1":
                print(date, line_data[1])
        elif identifier == "cpu_temp":
            cpu_temp_data["time"].append(date.timestamp())
            cpu_temp = float(line_data[1])
            cpu_temp_data["celsius"].append(cpu_temp)

    weights = np.polyfit(ina_data["time"], ina_data["voltage_V"], 3)
    model = np.poly1d(weights)

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ax1.plot(ina_data["time"], ina_data["voltage_V"])
    ax1.plot(ina_data["time"], model(ina_data["time"]))

    ax2.plot(ina_data["time"], ina_data["current_mA"], color='g')

    fig, ax3 = plt.subplots()
    ax3.plot(cpu_temp_data["time"], cpu_temp_data["celsius"])
    plt.show()


if __name__ == '__main__':
    main()

