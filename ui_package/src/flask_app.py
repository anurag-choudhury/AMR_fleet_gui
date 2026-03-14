#!/usr/bin/env python3

from flask import Flask, render_template, send_from_directory
import os
import yaml
import rclpy
from rclpy.node import Node

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_app_node')
        self.declare_parameter('app_address', '0.0.0.0')
        self.declare_parameter('port_app', 50505)

# Flask app setup
template_path = os.path.join(os.path.dirname(__file__), "templates")
static_path = os.path.join(os.path.dirname(__file__), "static")
config_path = os.path.join(os.path.dirname(__file__), "config")
ros_path = os.path.join(os.path.dirname(__file__), "ros")

app = Flask(__name__, template_folder=template_path)

# ✅ Pages
@app.route('/')
@app.route('/route')
@app.route('/control')
@app.route('/info')
def index():
    return render_template('index.html')

# ✅ Static assets (React build)
@app.route("/static/<path:filename>")
def serve_static(filename):
    return send_from_directory(static_path, filename)

# ✅ ROS scripts
@app.route("/ros/<path:filename>")
def serve_ros(filename):
    return send_from_directory(ros_path, filename)
# ✅ config files
@app.route("/config/<path:filename>")
def serve_config(filename):
    return send_from_directory(config_path, filename)

if __name__ == '__main__':
    rclpy.init()
    node = FlaskNode()

    local_ip = node.get_parameter('app_address').get_parameter_value().string_value
    local_port = node.get_parameter('port_app').get_parameter_value().integer_value

    try:
        app.run(host=local_ip, port=local_port)
    finally:
        rclpy.shutdown()
