from flask import Flask, jsonify

from server.container_monitor import ContainerMonitor
from server.hardware_monitor import HardwareMonitor

# TODO: This has to be in a cleaner file setup
# Create a Flask web server
app = Flask(__name__)
container_monitor = ContainerMonitor()

# Define a route to get Pi information


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
    running_controllers = container_monitor.get_all_running_containers(
        "controller")
    running_controllers = {"running_controllers": running_controllers}
    return jsonify(running_controllers)

@app.route('/controller/stop_all', methods=['POST'])
def stop_all_running_controllers():
    whitelist = ["rosbridge","rpi3-wifiap","dev_container"]
    container_monitor.stop_all_containers_except(whitelist)
    return  jsonify({"result": True}), 200
