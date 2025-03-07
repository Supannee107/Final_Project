#!/usr/bin/env python3
import psutil
import time
import csv
from datetime import datetime

# Function to get CPU and RAM usage
def get_system_usage():
    cpu_usage = psutil.cpu_percent(interval=None)  # Non-blocking
    ram_usage = psutil.virtual_memory().percent
    return cpu_usage, ram_usage

interval = 0.1  # 100 milliseconds

filename = 'system_usage.csv'

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Timestamp', 'CPU Usage (%)', 'RAM Usage (%)'])

    try:
        while True:
            start_time = time.time()

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # millisecond precision
            cpu, ram = get_system_usage()
            print(f"{timestamp} | CPU: {cpu}% | RAM: {ram}%")
            writer.writerow([timestamp, cpu, ram])

            elapsed = time.time() - start_time
            sleep_duration = interval - elapsed
            if sleep_duration > 0:
                time.sleep(sleep_duration)
    except KeyboardInterrupt:
        print(f"\nMonitoring stopped by user. Data saved in '{filename}'.")
