#!/usr/bin/env python3

from flask import Flask, render_template
import yaml
import rclpy
from rclpy.node import Node

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_app_node')
        self.declare_parameter('app_address', '127.0.0.1')
        self.declare_parameter('port_app', 50505)

app = Flask(__name__)

@app.route('/')
@app.route('/route')
@app.route('/control')
@app.route('/info')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    rclpy.init()
    node = FlaskNode()

    local_ip = node.get_parameter('app_address').get_parameter_value().string_value
    print(local_ip)
    local_port = node.get_parameter('port_app').get_parameter_value().integer_value

    try:
        app.run(host=local_ip, port=local_port)
    finally:
        rclpy.shutdown()
