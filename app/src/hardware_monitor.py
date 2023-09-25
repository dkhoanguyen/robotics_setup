#!/usr/bin/python3

import sys
import netifaces
import socket
import psutil
import platform

# Check if Pi Juice is available
try:
    from pijuice import PiJuicee
except ImportError:
    print("PiJuice module is not available.")


class HardwareMonitor(object):
    def __init__(self) -> None:
        pass

    @staticmethod
    def get_information():
        general_info = {
            'platform': platform.platform(),
            'architecture': platform.processor(),
            'ram': f"{psutil.virtual_memory().total / (1024 ** 3)} GB",
        }
        return general_info

    @staticmethod
    def get_status():
        # Temperature
        temperature = "Unavailable"
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as temp_file:
                temperature = temp_file.read().strip()
                temperature = f"{float(temperature) / 1000:.1f}°C"
        except Exception as e:
            f"Error: {str(e)}"
        # Total CPU usage
        cpu_usage = "Unavailable"
        try:
            cpu_usage = psutil.cpu_percent(interval=1)
            cpu_usage = f"CPU Usage: {cpu_usage}%"
        except Exception as e:
            cpu_usage = f"Error: {str(e)}"
        # Total RAM usage
        ram_usage = "Unavailable"
        try:
            ram = psutil.virtual_memory()
            ram_usage = f"Total: {ram.total / (1024 ** 3):.2f} GB, Used: {ram.used / (1024 ** 3):.2f} GB, Percentage: {ram.percent}%"
        except Exception as e:
            ram_usage = f"Error: {str(e)}"

        return {
            'temp': temperature,
            'cpu_usage': cpu_usage,
            'ram_usage': ram_usage
        }

    @staticmethod
    def get_battery_status():
        if "pijuice" not in sys.modules:
            return {
                "status": "Unavailable",
                "charge_level": "Unavailable",
                "voltage": "Unavailable"
            }
        pijuice = PiJuice(1, 0x14)
        battery_status = pijuice.status.GetStatus()
        battery_charge = pijuice.status.GetChargeLevel()
        battery_voltage = pijuice.status.GetBatteryVoltage()

        return {
            "status": battery_status,
            "charge_level": battery_charge,
            "voltage": battery_voltage
        }

    @staticmethod
    def get_all_ip_addresses():
        ipv4_addresses = []

        # Get the hostname of the local machine
        hostname = socket.gethostname()

        # Get the IP addresses associated with the hostname
        ip_addresses = socket.getaddrinfo(hostname, None)

        output = []
        for ip_info in ip_addresses:
            ip_address = ip_info[4][0]
            # Check if it's an IPv4 address
            if '.' in ip_address:
                output.append(ip_address)
        return {
            "ip_addresses": output
        }
