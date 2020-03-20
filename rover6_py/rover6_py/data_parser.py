import datetime
import pandas as pd
from lib.logger_manager import LoggerManager
from lib.data_logger import DataLogger
from lib.config import ConfigManager
import matplotlib.pyplot as plt

data_log_config = ConfigManager.get_data_log_config()

with open(data_log_config.path) as file:
    contents = file.read()

start_date = ""

ina_df = pd.DataFrame({
    "time": [],
    "voltage_V": [],
    "current_mA": [],
})
cpu_temp_df = pd.DataFrame({
    "time": [],
    "celcius": [],
})

raw_data = []
for line in contents.splitlines():
    date_str, msg = line.split(":\t", 1)
    date = datetime.datetime.strptime(date_str, "%Y-%m-%dT%H:%M:%S,%f")
    if msg == DataLogger.start_flag:
        print(line)
    raw_data.append((date, msg))

for date, msg in raw_data:
    line_data = msg.split("\t")
    identifier = line_data[0]
    if identifier == "ina":
        timestamp, voltage_V, current_mA = list(map(float, line_data[1:]))
        ina_df["time"].append(timestamp)
        ina_df["voltage_V"].append(voltage_V)
        ina_df["current_mA"].append(current_mA)
    elif identifier == "camera_detected":
        if line_data[1] != "supported=1 detected=1":
            print(date, line_data[1])
    elif identifier == "cpu_temp":
        cpu_temp_df["time"].append(date.timestamp())
        cpu_temp_df["celcius"].append(line_data[1])

plt.plot(ina_df["time"], ina_df["voltage_V"])
plt.show()
