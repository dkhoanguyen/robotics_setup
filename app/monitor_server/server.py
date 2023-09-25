import os
import psutil
import platform
from flask import Flask, jsonify

from hardware_monitor import HardwareMonitor

# Create a Flask web server
app = Flask(__name__)

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

# Run the server
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
