// import ROSLIBB from 'roslib'
window.NAV2D = window.NAV2D || {};

window.NAV2D.pointsArray = [];
window.NAV2D.pointsFromTopic = [];
window.NAV2D.arePointsSettable = false;
window.NAV2D.canvas = null;
window.NAV2D.pointType = null;
window.NAV2D.finishedPointItem = null;
window.NAV2D.orientatedPointItem = null;

window.NAV2D.mapInited = false;
window.NAV2D.scale = { x: 0, y: 0 };


window.NAV2D.checkScale = () => {
  // Your code to be executed periodically
  // console.log("check scale is called")
  if (
    window.NAV2D.scale.x != window.NAV2D.canvas.scene.scaleX ||
    window.NAV2D.scale.y != window.NAV2D.canvas.scene.scaleY
  ) {
    window.NAV2D.scale.x = window.NAV2D.canvas.scene.scaleX;
    window.NAV2D.scale.y = window.NAV2D.canvas.scene.scaleY;
    window.NAV2D.pointsArray = drawPoints(
      window.NAV2D.pointsFromTopic,
      window.NAV2D.canvas.scene,
    );
  }
};

// Call the function every 1 second (1000 milliseconds)
const intervalId = setInterval(window.NAV2D.checkScale, 100);

window.NAV2D.InitMap = (ros) => {
  console.log("Initializing map...");
  const topic = window.AppConfig.NAV2_MAP_TOPIC;

  /* Setup a client to get the map */
  const client = new window.ROS2D.OccupancyGridClient({
    ros,
    rootObject: window.NAV2D.canvas.scene,
    continuous: true,
    topic,
    messageType: "nav_msgs/msg/OccupancyGrid",
  });

  console.log("OccupancyGridClient created:", client);

  client.on("change", () => {
    console.log("Map updated. Current grid data:", client.currentGrid);

    if (!client.currentGrid) {
      console.error("No grid data received from /map topic.");
      return;
    }

    // Scale and shift the map correctly
    window.NAV2D.canvas.scaleToDimensions(
      client.currentGrid.width,
      client.currentGrid.height,
    );
    window.NAV2D.canvas.shift(
      client.currentGrid.pose.position.x,
      client.currentGrid.pose.position.y,
    );

    // Log map pose for debugging
    console.log("Map Pose:", client.currentGrid.pose);
  });

  if (!window.NAV2D.mapInited) {
    window.NAV2D.mapInited = true;

    customnavigator(ros);
    init_pose_fn(ros)
  }
};
/* Cleaning map */
window.NAV2D.ClearMap = () => {
  window.NAV2D.pointsArray.forEach((marker) =>
    window.NAV2D.canvas.scene.removeChild(marker),
  );
  window.NAV2D.pointsArray = [];
};

const drawPoints = (points, canvas) => {
  if (!(points && canvas)) return;
  window.NAV2D.ClearMap();

  window.NAV2D.pointsArray = points.map((point) => {
    const defaultPointItem = serializePoint(point, canvas);
    canvas.addChild(defaultPointItem);
    return defaultPointItem;
  });
  return window.NAV2D.pointsArray;
};

const customnavigator = (ros) => {
  if (!ros || !window.NAV2D.canvas) {
    console.error("ROS or canvas is not initialized");
    return;
  }

  const canvas = window.NAV2D.canvas.scene;
  /* Send message about map initialization */
  console.log(window.AppConfig.UI_MESSAGE_TOPIC)
  const messageTopic = new window.ROSLIB.Topic({
    ros,
    name: window.AppConfig.UI_MESSAGE_TOPIC,
    messageType: "std_msgs/String",
  });

  messageTopic.publish(
    new window.ROSLIB.Message({ data: "Initialization completed" }),
  );


  let robotPose = { x: 0, y: 0, theta: 0 };


  // Subscribe to robot odometry to update pose
  const odomTopic = new ROSLIB.Topic({
    ros: ros,
    name: window.AppConfig.ROBOT_POSE_TOPIC,
    messageType: "nav_msgs/Odometry",
  });
  const quaternionToTheta = (qx, qy, qz, qw) => {
    return Math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  };


  const tfListener = new window.ROSLIB.Topic({
    ros: ros,
    name: "/tf",
    messageType: "tf2_msgs/TFMessage",
  });
  // console.log(window.AppConfig.NAV2_MAP_FRAME)
  let mapToOdom = null;
  let odomToBaseLink = null;

  tfListener.subscribe((message) => {
    message.transforms.forEach((transform) => {
      if (transform.header.frame_id === window.AppConfig.NAV2_MAP_FRAME && transform.child_frame_id === window.AppConfig.ROBOT_POSE_FRAME) {
        mapToOdom = transform.transform;
      }
      if (transform.header.frame_id === window.AppConfig.ROBOT_POSE_FRAME && transform.child_frame_id === window.AppConfig.ROBOT_BASE_FRAME) {
        odomToBaseLink = transform.transform;
      }
    });

    if (mapToOdom && odomToBaseLink) {
      // Extract translation and rotation from map -> odom
      const x_map = mapToOdom.translation.x;
      const y_map = mapToOdom.translation.y;
      const theta_map = quaternionToTheta(
        mapToOdom.rotation.x,
        mapToOdom.rotation.y,
        mapToOdom.rotation.z,
        mapToOdom.rotation.w
      );

      // Extract translation and rotation from odom -> base_link
      const x_odom = odomToBaseLink.translation.x;
      const y_odom = odomToBaseLink.translation.y;
      const theta_odom = quaternionToTheta(
        odomToBaseLink.rotation.x,
        odomToBaseLink.rotation.y,
        odomToBaseLink.rotation.z,
        odomToBaseLink.rotation.w
      );

      // Compute final position (map → base_link transformation)
      const x_final = x_map + x_odom * Math.cos(theta_map) - y_odom * Math.sin(theta_map);
      const y_final = y_map + x_odom * Math.sin(theta_map) + y_odom * Math.cos(theta_map);
      let theta_final = theta_map + theta_odom;

      // Normalize angle between -π and π
      theta_final = Math.atan2(Math.sin(theta_final), Math.cos(theta_final));
      const theta_quat = thetaToQuaternion(theta_final)
      // Update Robot Marker
      robotMarker.x = x_final;
      robotMarker.y = -y_final; // Flip Y as needed
      robotMarker.scaleX = 1.0 / canvas.scaleX;
      robotMarker.scaleY = 1.0 / canvas.scaleY;
      robotMarker.rotation = canvas.rosQuaternionToGlobalTheta(theta_quat); // Fix orientation flip
      // robotMarker.rotation = theta_final; // Fix orientation flip
      robotMarker.visible = true;


    }
    function thetaToQuaternion(theta) {
      // Normalize theta to the range [-π, π]
      const thetaNormalized = Math.atan2(Math.sin(theta), Math.cos(theta));

      // Convert to quaternion
      return {
        x: 0,
        y: 0,
        z: Math.sin(thetaNormalized / 2),
        w: Math.cos(thetaNormalized / 2),
      };
    }
  });

  // Subscribe to Odometry topic
  odomTopic.subscribe((odom) => {
    robotPose.x = odom.pose.pose.position.x;
    robotPose.y = odom.pose.pose.position.y;
    robotPose.theta = getYawFromQuat(odom.pose.pose.orientation);
  });


  // Function to extract yaw from quaternion
  function getYawFromQuat(q) {
    let siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny_cosp, cosy_cosp);
  }


  createSubscribeTopic(
    ros,
    "/WayPoints_topic",
    "ui_package/ArrayPoseStampedWithCovariance",
    (data) => {
      if (!Array.isArray(data.poses)) {
        console.error("WayPoints_topic data.poses is not an array");
        return;
      }
      window.NAV2D.pointsFromTopic = data.poses;
      window.NAV2D.pointsArray = drawPoints(data.poses, canvas);
      window.NAV2D.mapInited = false;
    },
  );

  /* ROBOT MARKER SECTION: */
  const robotMarker = createCanvasPoint(25, {
    r: 0,
    g: 0,
    b: 255,
    a: 1,
  });
  robotMarker.visible = false;
  canvas.addChild(robotMarker);

  /* Robot position watcher */
  createSubscribeTopic(ros, window.AppConfig.ROBOT_POSE_TOPIC, "nav_msgs/msg/Odometry", (data) => {
    // console.log("i am working")
    const pose = data.pose.pose;
    // robotMarker.x = pose.position.x;
    // robotMarker.y = -pose.position.y;
    // robotMarker.scaleX = 1.0 / canvas.scaleX;
    // robotMarker.scaleY = 1.0 / canvas.scaleY;
    // robotMarker.rotation = canvas.rosQuaternionToGlobalTheta(pose.orientation);
    // robotMarker.visible = true;
  });

  /* MOUSE EVENT SECTION */
  let isMousePressed = false;
  let isMouseMoved = false;
  let positionVectorItem = null;
  let orientationMarker = null;

  const handleMouseDown = (event) => {
    isMousePressed = true;
    const positionItem = canvas.globalToRos(event.stageX, event.stageY);
    positionVectorItem = new window.ROSLIB.Vector3(positionItem);
  };

  const handleMouseMove = (event) => {
    if (!isMousePressed) return;
    isMouseMoved = true;
    canvas.removeChild(orientationMarker);

    const currentPos = canvas.globalToRos(event.stageX, event.stageY);
    const currentPositionVectorItem = new window.ROSLIB.Vector3(currentPos);

    orientationMarker = createCanvasPoint(25, {
      r: 0,
      g: 255,
      b: 0,
      a: 1,
    });

    const xDelta = currentPositionVectorItem.x - positionVectorItem.x;
    const yDelta = currentPositionVectorItem.y - positionVectorItem.y;
    const thetaRadians = Math.atan2(xDelta, yDelta);
    let thetaDegrees = thetaRadians * (180.0 / Math.PI);

    if (thetaDegrees >= 0 && thetaDegrees <= 180) {
      thetaDegrees += 270;
    } else {
      thetaDegrees -= 90;
    }

    orientationMarker.x = positionVectorItem.x;
    orientationMarker.y = -positionVectorItem.y;
    orientationMarker.rotation = thetaDegrees;
    orientationMarker.scaleX = 1.0 / canvas.scaleX;
    orientationMarker.scaleY = 1.0 / canvas.scaleY;
    canvas.addChild(orientationMarker);
  };

  const handleMouseUp = (event) => {
    if (!isMousePressed) return;
    if (!isMouseMoved) {
      console.error("Please, set the direction of the WayPoint");
      messageTopic.publish(
        new window.ROSLIB.Message({
          data: "Please, set the direction of the WayPoint",
        }),
      );
      return;
    }

    isMousePressed = false;
    isMouseMoved = false;

    let pointColor = {};
    /* Logic for different point types */
    /*
    if (window.NAV2D.pointType === "navigate") {
      pointColor = {
        r: 255,
        g: 0,
        b: 0,
        a: 1,
      };
    } else if (window.NAV2D.pointType === "home") {
      pointColor = {
        r: 124,
        g: 252,
        b: 0,
        a: 1,
      };
    } else if (window.NAV2D.pointType === "charge") {
      pointColor = {
        r: 186,
        g: 85,
        b: 211,
        a: 1,
      };
    } else {
      console.error("Point type was not selected");
      messageTopic.publish(
        new window.ROSLIB.Message({ data: "Point type was not selected" }),
      );
      return;
    }
    */
    pointColor = {
      r: 255,
      g: 0,
      b: 0,
      a: 1,
    };
    const goalMarkerItem = createCanvasPoint(15, pointColor);

    goalMarkerItem.x = orientationMarker.x;
    goalMarkerItem.y = orientationMarker.y;
    goalMarkerItem.rotation = orientationMarker.rotation;
    goalMarkerItem.scaleX = orientationMarker.scaleX;
    goalMarkerItem.scaleY = orientationMarker.scaleY;

    window.NAV2D.orientatedPointItem = goalMarkerItem;
    window.NAV2D.pointsArray.push(goalMarkerItem);
    canvas.addChild(goalMarkerItem);

    const goalPos = canvas.globalToRos(event.stageX, event.stageY);
    const goalPosVec3 = new window.ROSLIB.Vector3(goalPos);
    const xDelta = goalPosVec3.x - positionVectorItem.x;
    const yDelta = goalPosVec3.y - positionVectorItem.y;
    let thetaRadians = calculateThetaRadians(xDelta, yDelta);

    const qz = Math.sin(-thetaRadians / 2.0);
    const qw = Math.cos(-thetaRadians / 2.0);

    const orientation = new window.ROSLIB.Quaternion({
      x: 0,
      y: 0,
      z: qz,
      w: qw,
    });

    const pose = new window.ROSLIB.Pose({
      position: positionVectorItem,
      orientation,
    });
    window.NAV2D.finishedPointItem = pose;
    canvas.removeChild(orientationMarker);
  };

  const handleCanvasEvent = (event, mouseEventType) => {
    if (!window.NAV2D.arePointsSettable) {
      return;
    }

    switch (mouseEventType) {
      case "down":
        handleMouseDown(event);
        break;
      case "move":
        handleMouseMove(event);
        break;
      case "up":
        handleMouseUp(event);
        break;
    }
  };

  const onCanvasMove = (event) => {
    handleCanvasEvent(event, "move");
  };

  canvas.addEventListener("stagemousedown", (event) => {
    handleCanvasEvent(event, "down");
    canvas.addEventListener("stagemousemove", onCanvasMove);
  });

  canvas.addEventListener("stagemouseup", (event) => {
    handleCanvasEvent(event, "up");
    canvas.removeEventListener("stagemousemove", onCanvasMove);
  });
};

const createSubscribeTopic = (ros, name, messageType, callback) => {
  const topicObject = {
    ros,
    name,
    messageType,
  };

  if (name === "/odom") {
    topicObject.throttle_rate = 1;
  }
  const topic = new window.ROSLIB.Topic(topicObject);
  topic.subscribe(callback);
  return topic;
};

const calculateThetaRadians = (xDelta, yDelta) => {
  let thetaRadians = Math.atan2(xDelta, yDelta);
  if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
    thetaRadians += (3 * Math.PI) / 2;
  } else {
    thetaRadians -= Math.PI / 2;
  }
  return thetaRadians;
};

const createCanvasPoint = (size, color) => {
  return new window.ROS2D.NavigationArrow({
    size,
    strokeSize: 1,
    fillColor: window.createjs.Graphics.getRGB(
      color.r,
      color.g,
      color.b,
      color.a,
    ),
    pulse: false,
  });
};

window.NAV2D.sendPointToRobot = (ros, time) => {
  const pointDetails = window.NAV2D.finishedPointItem;
  console.log(pointDetails)
  const { hours, minutes } = time;

  const wayPoint = new window.ROSLIB.Topic({
    ros,
    name: "/new_way_point",
    messageType: "geometry_msgs/PoseWithCovarianceStamped",
  });

  const sendDataArray = [
    0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  ];
  /*
  if (window.NAV2D.pointType === "navigate") {
    sendDataArray[0] = 3;
  } else if (window.NAV2D.pointType === "home") {
    sendDataArray[0] = 1;
  } else if (window.NAV2D.pointType === "charge") {
    sendDataArray[0] = 2;
  }
  */
  sendDataArray[0] = 3;
  sendDataArray[1] = Number(hours) || 0;
  sendDataArray[2] = Number(minutes) || 0;

  const messageObject = {
    header: { frame_id: window.AppConfig.NAV2_MAP_FRAME },
    pose: {
      pose: {
        position: {
          x: pointDetails.position.x,
          y: pointDetails.position.y,
          z: 0.0,
        },
        orientation: {
          z: pointDetails.orientation.z,
          w: pointDetails.orientation.w,
        },
      },
      covariance: sendDataArray,
    },
  };

  const poseMessage = new window.ROSLIB.Message(messageObject);

  wayPoint.publish(poseMessage);
  window.NAV2D.finishedPointItem = null;
};

const serializePoint = (point, canvas) => {
  const defaultPointItem = createCanvasPoint(15, {
    r: 255,
    g: 0,
    b: 0,
    a: 1,
  });

  defaultPointItem.x = point.pose.pose.position.x;
  defaultPointItem.y = -point.pose.pose.position.y;
  defaultPointItem.rotation = canvas.rosQuaternionToGlobalTheta(
    point.pose.pose.orientation,
  );
  defaultPointItem.scaleX = 1.0 / canvas.scaleX;
  defaultPointItem.scaleY = 1.0 / canvas.scaleY;

  return defaultPointItem;
};
const init_pose_fn = (ros) => {
  // Variables for Initialize Pose
  // let isInitialPoseMode = false;
  let initialPoseMousePressed = false;
  let initialPoseMouseMoved = false;
  let initialPosePositionVectorItem = null;
  let initialPoseOrientationMarker = null;
  console.log(window.NAV2D)
  const canvas = window.NAV2D.canvas.scene

  // Mouse down handler for Initialize Pose
  const handleInitialPoseMouseDown = (event) => {
    console.log(window.isInitialPoseMode)
    if (!window.isInitialPoseMode) return; // Check the global flag
    initialPoseMousePressed = true;
    const positionItem = canvas.globalToRos(event.stageX, event.stageY);
    initialPosePositionVectorItem = new window.ROSLIB.Vector3(positionItem);
  };

  const handleInitialPoseMouseMove = (event) => {
    if (!initialPoseMousePressed || !window.isInitialPoseMode) return; // Check the global flag
    initialPoseMouseMoved = true;

    // Remove previous orientation marker (if any)
    if (initialPoseOrientationMarker) {
      canvas.removeChild(initialPoseOrientationMarker);
    }

    const currentPos = canvas.globalToRos(event.stageX, event.stageY);
    const currentPositionVectorItem = new window.ROSLIB.Vector3(currentPos);

    initialPoseOrientationMarker = createCanvasPoint(25, {
      r: 0,
      g: 255,
      b: 0,
      a: 1,
    });

    const xDelta = currentPositionVectorItem.x - initialPosePositionVectorItem.x;
    const yDelta = currentPositionVectorItem.y - initialPosePositionVectorItem.y;
    const thetaRadians = Math.atan2(xDelta, yDelta);
    let thetaDegrees = thetaRadians * (180.0 / Math.PI);

    if (thetaDegrees >= 0 && thetaDegrees <= 180) {
      thetaDegrees += 270;
    } else {
      thetaDegrees -= 90;
    }

    initialPoseOrientationMarker.x = initialPosePositionVectorItem.x;
    initialPoseOrientationMarker.y = -initialPosePositionVectorItem.y;
    initialPoseOrientationMarker.rotation = thetaDegrees;
    initialPoseOrientationMarker.scaleX = 1.0 / canvas.scaleX;
    initialPoseOrientationMarker.scaleY = 1.0 / canvas.scaleY;
    canvas.addChild(initialPoseOrientationMarker);
  };

  const handleInitialPoseMouseUp = (event) => {
    if (!initialPoseMousePressed || !window.isInitialPoseMode) return; // Check the global flag
    initialPoseMousePressed = false;
    initialPoseMouseMoved = false;

    const goalPos = canvas.globalToRos(event.stageX, event.stageY);
    const goalPosVec3 = new window.ROSLIB.Vector3(goalPos);
    const xDelta = goalPosVec3.x - initialPosePositionVectorItem.x;
    const yDelta = goalPosVec3.y - initialPosePositionVectorItem.y;
    let thetaRadians = calculateThetaRadians(xDelta, yDelta);

    const qz = Math.sin(-thetaRadians / 2.0);
    const qw = Math.cos(-thetaRadians / 2.0);

    const orientation = new window.ROSLIB.Quaternion({
      x: 0,
      y: 0,
      z: qz,
      w: qw,
    });

    const pose = new window.ROSLIB.Pose({
      position: initialPosePositionVectorItem,
      orientation,
    });

    // Publish the initial pose
    publishInitialPose(pose);

    // Reset mode and clean up
    window.isInitialPoseMode = false; // Reset the global flag
    canvas.removeChild(initialPoseOrientationMarker);
  };

  // Attach event listeners for Initialize Pose
  canvas.addEventListener('stagemousedown', (event) => {
    handleInitialPoseMouseDown(event);
    canvas.addEventListener('stagemousemove', handleInitialPoseMouseMove);
  });

  canvas.addEventListener('stagemouseup', (event) => {
    handleInitialPoseMouseUp(event);
    canvas.removeEventListener('stagemousemove', handleInitialPoseMouseMove);
  });

  const publishInitialPose = async (pose) => {
    // Function to get the latest TF timestamp
    const getLatestTime = async () => {
      return new Promise((resolve) => {
        const tfTopic = new window.ROSLIB.Topic({
          ros: ros,
          name: '/tf',
          messageType: 'tf2_msgs/TFMessage',
        });

        tfTopic.subscribe((message) => {
          if (message.transforms.length > 0) {
            const latestTransform = message.transforms[0];
            resolve(latestTransform.header.stamp);
          }
          tfTopic.unsubscribe();
        });
      });
    };

    const latestTime = await getLatestTime();
    const sec = latestTime.sec;
    const nanosec = latestTime.nanosec;

    const covariance = [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    ];

    // Create a publisher for /initialpose
    const initialPoseTopic = new window.ROSLIB.Topic({
      ros: ros,
      name: window.AppConfig.NAV2_INITIAL_POSE_TOPIC,
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
    });

    // Create the initial pose message
    const initialPoseMessage = new window.ROSLIB.Message({
      header: {
        frame_id: 'map',
        stamp: { sec, nanosec },
      },
      pose: {
        pose: pose,
        covariance: covariance,
      },
    });

    // Publish the initial pose to /initialpose
    initialPoseTopic.publish(initialPoseMessage);
    console.log("Initial pose published:", initialPoseMessage);
  };





}