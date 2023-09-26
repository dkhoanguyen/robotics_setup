import os
import dbus
from flask import Flask, jsonify, request

from server.container_monitor import ContainerMonitor
from server.hardware_monitor import HardwareMonitor

# TODO: This has to be in a cleaner file setup
# Create a Flask web server
app = Flask(__name__)


@app.route('/shutdown', methods=['POST'])
def request_shutdown():
    bus = dbus.SystemBus()
    systemd1 = bus.get_object('org.freedesktop.systemd1', '/org/freedesktop/systemd1')
    # Get the Manager interface
    manager = dbus.Interface(systemd1, 'org.freedesktop.systemd1.Manager')
    # Trigger a system reboot
    manager.PowerOff()
    return jsonify({"message": "Computer is shutting down..."}), 200



@app.route('/restart', methods=['POST'])
def request_restart():
    bus = dbus.SystemBus()
    systemd1 = bus.get_object('org.freedesktop.systemd1', '/org/freedesktop/systemd1')

    # Get the Manager interface
    manager = dbus.Interface(systemd1, 'org.freedesktop.systemd1.Manager')

    # Trigger a system shutdown
    manager.Reboot()
    return jsonify({"message": "Computer is shutting down..."}), 200


@app.route('/hardware-info', methods=['GET'])
def get_info():
    # Get Pi information
    info = HardwareMonitor.get_information()
    return jsonify(info)


@app.route('/hardware-status',  methods=['GET'])
def get_status():
    status = HardwareMonitor.get_status()
    return jsonify(status)


@app.route('/battery-status', methods=['GET'])
def get_battery_status():
    status = HardwareMonitor.get_battery_status()
    return jsonify(status)


@app.route('/ip-address', methods=['GET'])
def get_ip_address():
    addess = HardwareMonitor.get_all_ip_addresses()
    return jsonify(addess)


@app.route('/controller/current', methods=['GET'])
def get_all_available_controller():
    container_monitor = ContainerMonitor()
    running_controllers = container_monitor.get_all_running_containers(
        "controller")
    running_controllers = {"running_controllers": running_controllers}
    return jsonify(running_controllers)


@app.route('/controller/stop_all', methods=['POST'])
def stop_all_running_controllers():
    whitelist = ["rosbridge", "rpi3-wifiap", "dev_container"]
    container_monitor = ContainerMonitor()
    container_monitor.stop_all_containers_except(whitelist)
    return jsonify({"result": True}), 200


@app.route('/controller/start', methods=['POST'])
def start_controller():
    container_monitor = ContainerMonitor()
    controller_config = request.json
    if controller_config["name"] == "ur3e":
        robot_ip = controller_config["params"]["robot_ip"]
        directory_path = f"{os.path.expanduser('~')}/git/robotics_setup/calibration_file"
        if not os.path.exists(directory_path) or not os.path.isdir(directory_path):
            os.makedirs(directory_path)
        # Start calibration
        container_config = {}
        container_config["image"] = "robotic_base:latest"
        container_config["name"] = "ur3e_calibration"
        container_config["tty"] = True
        container_config["privileged"] = True
        container_config["remove"] = True
        container_config["network"] = "host"
        container_config["volumes"] = {
            f"{os.path.expanduser('~')}/git/robotics_setup/calibration_file": {
                "bind": "/calibration_file",
                "mode": "rw"
            }
        }
        container_config["command"] = f"bash -c 'source /opt/ros/noetic/setup.bash && \
             source /ur_ws/devel/setup.bash && \
             timeout 20 roslaunch ur_calibration calibration_correction.launch \
            robot_ip:={robot_ip} target_filename:=/calibration_file/ur3e_calibration.yaml'"
        container_monitor.create_and_start_container(container_config)

        # Start Robot Controller
        container_config = {}
        container_config["image"] = "robotic_base:latest"
        container_config["name"] = "ur3e_controller"
        container_config["detach"] = True
        container_config["tty"] = True
        container_config["privileged"] = True
        container_config["network"] = "host"
        container_config["restart_policy"] = {"Name": "always"}
        container_config["volumes"] = {
            f"{os.path.expanduser('~')}/git/robotics_setup/calibration_file": {
                "bind": "/calibration_file",
                "mode": "rw"
            }
        }
        container_config["environment"] = {
            "ROS_MASTER_URI": "http://localhost:11311",
            "ROS_IP": "192.168.27.1"
        }
        container_config["command"] = f"bash -c 'source /opt/ros/noetic/setup.bash && \
             source /ur_ws/devel/setup.bash && \
             sleep 15 && \
             roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:={robot_ip} \
             kinematics_config:=/calibration_file/ur3e_calibration.yaml'"
        container_monitor.create_and_start_container(container_config)
    return ""
