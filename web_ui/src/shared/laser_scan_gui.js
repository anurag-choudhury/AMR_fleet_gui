import ROSLIB from "roslib";
// const ROSLIB = require(roslib)

function displayLaserScan() {
    console.log("hell0")
    let marker_radius = 0.03;
    let marker_fill_color = createjs.Graphics.getRGB(255, 0, 0, 1.0);
    let laser_listener = new ROSLIB.Topic({
        ros: ros,
        name: ROS_CONFIG.LASER_TOPIC,
        messageType: 'sensor_msgs/LaserScan'
    });
    let prev_markers = null;
    const tfClient = new ROSLIB.TFClient({
        ros: ros,
        fixedFrame: "map", // Map frame
        angularThres: 0.01,
        transThres: 0.01,
    });

    tfClient.subscribe("base_link", (transform) => {
        console.log("subscribed tf_clinet")
        // Apply the transform to the robot's pose
        robotPose.x = transform.translation.x;
        robotPose.y = transform.translation.y;
        robotPose.theta = transform.rotation.z; // Yaw in radians
    });
    laser_listener.subscribe(function (msg) {
        const num = msg.ranges.length
        const angles = Array.from({ length: num }, (_, i) => msg.angle_min + (msg.angle_max - msg.angle_min) / num * i);
        const poses_2d = angles.flatMap((angle, index) => {
            const range = msg.ranges[index];
            if (range > msg.range_min && range < msg.range_max) {
                return [[Math.cos(angle) * range, Math.sin(angle) * range, -angle]]
            }
            return []  // Skip this point
        });
        if (base_footprint_tf === null) {
            console.log('no tf');
            return;
        }
        // TODO: We might be able to apply the tf transform to the container itself, and dont have to do it on each pose.
        // Init the graphics component
        const scan_markers = new createjs.Container();
        const graphics = new createjs.Graphics().beginFill(marker_fill_color).drawCircle(0, 0, marker_radius).endFill();

        // Transform each point and add it to the graphics
        poses_2d.forEach(pt => {
            // pt[2] += Math.PI / 2
            const pose = new ROSLIB.Pose({
                position: new ROSLIB.Vector3({
                    x: pt[0], y: pt[1], z: 0
                }), orientation: new ROSLIB.Quaternion({
                    x: 0, y: 0, z: Math.cos(pt[2]), w: Math.sin(pt[2])

                })
            });
            pose.applyTransform(base_footprint_tf)
            const marker = new createjs.Shape(graphics)
            marker.x = pose.position.x;
            marker.y = -pose.position.y;
            marker.rotation = - getYawFromQuat(pose.orientation).toFixed(2);
            scan_markers.addChild(marker)
        });

        // TODO: Just update the old one, dont make new ones everytime
        if (this.prev_markers !== null) {
            viewer.scene.removeChild(prev_markers);
        }

        viewer.addObject(scan_markers);
        prev_markers = scan_markers;
    });
}
// displayLaserScan()
export default displayLaserScan();