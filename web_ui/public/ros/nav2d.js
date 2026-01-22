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
window.NAV2D.tf = window.NAV2D.tf || {
  // store latest transforms by "parent->child"
  latest: {},
};
window.NAV2D._wpHandlers = window.NAV2D._wpHandlers || {
  attachedTo: null,
  down: null,
  move: null,
  up: null,
};

window.NAV2D.ensureWaypointMouseHandlers = () => {
  const canvas = window.NAV2D?.canvas?.scene;
  if (!canvas) return;

  // If already attached to THIS exact scene, do nothing
  if (window.NAV2D._wpHandlers.attachedTo === canvas) return;

  // If attached to an old scene, remove old listeners
  const old = window.NAV2D._wpHandlers.attachedTo;
  if (old) {
    try {
      old.removeEventListener("stagemousedown", window.NAV2D._wpHandlers.down);
      old.removeEventListener("stagemousemove", window.NAV2D._wpHandlers.move);
      old.removeEventListener("stagemouseup", window.NAV2D._wpHandlers.up);
    } catch (e) { }
  }

  // Local state (per attachment)
  let isMousePressed = false;
  let isMouseMoved = false;
  let positionVectorItem = null;
  let orientationMarker = null;

  const down = (event) => {
    if (!window.NAV2D.arePointsSettable) return;

    isMousePressed = true;
    isMouseMoved = false;

    const pos = canvas.globalToRos(event.stageX, event.stageY);
    positionVectorItem = new window.ROSLIB.Vector3(pos);
  };

  const move = (event) => {
    if (!window.NAV2D.arePointsSettable) return;
    if (!isMousePressed) return;

    isMouseMoved = true;

    if (orientationMarker) {
      try { canvas.removeChild(orientationMarker); } catch (e) { }
      orientationMarker = null;
    }

    const currentPos = canvas.globalToRos(event.stageX, event.stageY);
    const cur = new window.ROSLIB.Vector3(currentPos);

    orientationMarker = createCanvasPoint(25, { r: 0, g: 255, b: 0, a: 1 });

    const xDelta = cur.x - positionVectorItem.x;
    const yDelta = cur.y - positionVectorItem.y;

    const thetaRadians = Math.atan2(xDelta, yDelta);
    let thetaDegrees = thetaRadians * (180.0 / Math.PI);
    if (thetaDegrees >= 0 && thetaDegrees <= 180) thetaDegrees += 270;
    else thetaDegrees -= 90;

    orientationMarker.x = positionVectorItem.x;
    orientationMarker.y = -positionVectorItem.y;
    orientationMarker.rotation = thetaDegrees;
    orientationMarker.scaleX = 1.0 / canvas.scaleX;
    orientationMarker.scaleY = 1.0 / canvas.scaleY;

    canvas.addChild(orientationMarker);
  };

  const up = (event) => {
    if (!window.NAV2D.arePointsSettable) return;
    if (!isMousePressed) return;

    isMousePressed = false;

    if (!isMouseMoved) {
      console.log("⚠️ Drag to set orientation (click+drag).");
      return;
    }

    // direction based on drag end
    const endPos = canvas.globalToRos(event.stageX, event.stageY);
    const endVec = new window.ROSLIB.Vector3(endPos);

    const xDelta = endVec.x - positionVectorItem.x;
    const yDelta = endVec.y - positionVectorItem.y;
    const thetaRadians = calculateThetaRadians(xDelta, yDelta);

    const qz = Math.sin(-thetaRadians / 2.0);
    const qw = Math.cos(-thetaRadians / 2.0);

    const orientation = new window.ROSLIB.Quaternion({ x: 0, y: 0, z: qz, w: qw });

    const pose = new window.ROSLIB.Pose({
      position: positionVectorItem,
      orientation,
    });

    window.NAV2D.finishedPointItem = pose; // ✅ SETS for sendPointToRobot()

    // remove orientation marker
    if (orientationMarker) {
      try { canvas.removeChild(orientationMarker); } catch (e) { }
      orientationMarker = null;
    }

    console.log("✅ Waypoint set", {
      x: pose.position.x,
      y: pose.position.y,
      z: pose.orientation.z,
      w: pose.orientation.w,
    });
  };

  canvas.addEventListener("stagemousedown", down);
  canvas.addEventListener("stagemousemove", move);
  canvas.addEventListener("stagemouseup", up);

  window.NAV2D._wpHandlers.attachedTo = canvas;
  window.NAV2D._wpHandlers.down = down;
  window.NAV2D._wpHandlers.move = move;
  window.NAV2D._wpHandlers.up = up;

  console.log("✅ Waypoint mouse handlers attached to current scene");
};

window.NAV2D.laser = window.NAV2D.laser || {
  shape: null,          // createjs.Shape for scan points
  lastPoints: [],       // last scan points in MAP frame
  topic: "scan",
  enabled: true,
  dotRadius: 0.03,      // meters (map units)
  maxPoints: 2500,      // safety cap
  frame: 'laser',          // laser frame id from message
  scanTopic: "/scan",   // change if needed
};

window.NAV2D.scale = { x: 0, y: 0 };
window.NAV2D.ensureRobotMarker = () => {
  const scene = window.NAV2D?.canvas?.scene;
  if (!scene) return null;

  // If marker exists but is attached to an old scene, re-attach
  if (window.NAV2D.robotMarker && window.NAV2D.robotMarker.parent !== scene) {
    try { window.NAV2D.robotMarker.parent?.removeChild(window.NAV2D.robotMarker); } catch (e) { }
    scene.addChild(window.NAV2D.robotMarker);
  }

  // If marker doesn't exist, create and attach
  if (!window.NAV2D.robotMarker) {
    window.NAV2D.robotMarker = new window.ROS2D.NavigationArrow({
      size: 25,
      strokeSize: 1,
      fillColor: window.createjs.Graphics.getRGB(0, 0, 255, 1),
      pulse: false,
    });
    window.NAV2D.robotMarker.visible = false;
    scene.addChild(window.NAV2D.robotMarker);
  }

  // keep constant size on zoom
  window.NAV2D.robotMarker.scaleX = 1.0 / (scene.scaleX || 1.0);
  window.NAV2D.robotMarker.scaleY = 1.0 / (scene.scaleY || 1.0);

  return window.NAV2D.robotMarker;
};


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
    drawLaserPoints(window.NAV2D.canvas.scene);

  }
};

// Call the function every 1 second (1000 milliseconds)
const intervalId = setInterval(window.NAV2D.checkScale, 400);


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

    // initLaserScanOverlay_ScanOnly(ros);
    initLaserScanOverlay(ros);
    window.NAV2D.ensureWaypointMouseHandlers();
    window.NAV2D.onMapTabActivated();


  }
};
/* Cleaning map */
window.NAV2D.ClearMap = () => {
  window.NAV2D.pointsArray.forEach((marker) =>
    window.NAV2D.canvas.scene.removeChild(marker),
  );
  window.NAV2D.pointsArray = [];
};
window.NAV2D.onMapTabActivated = () => {
  console.log("🟢 Map tab activated: reattach overlays");

  // ensure robot marker is attached to new scene
  window.NAV2D.ensureRobotMarker?.();

  // reattach laser markers / redraw cached scan
  window.NAV2D?.laser?.redraw?.();

  // rescale existing points
  window.NAV2D.checkScale?.();
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
const quatToYaw = (q) => {
  // q: {x,y,z,w}
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny_cosp, cosy_cosp);
};

const tfKey = (parent, child) => `${parent}->${child}`;

const storeTF = (parent, child, transform) => {
  // transform: geometry_msgs/Transform
  window.NAV2D.tf.latest[tfKey(parent, child)] = {
    x: transform.translation.x,
    y: transform.translation.y,
    yaw: quatToYaw(transform.rotation),
    stamp: Date.now(),
  };
};

const getTF = (parent, child) => window.NAV2D.tf.latest[tfKey(parent, child)] || null;

// Compose T = A ∘ B (apply B then A)
const compose2D = (A, B) => {
  // A,B: {x,y,yaw}
  const c = Math.cos(A.yaw), s = Math.sin(A.yaw);
  const x = A.x + (B.x * c - B.y * s);
  const y = A.y + (B.x * s + B.y * c);
  let yaw = A.yaw + B.yaw;
  yaw = Math.atan2(Math.sin(yaw), Math.cos(yaw));
  return { x, y, yaw };
};

const transformPoint2D = (T, px, py) => {
  const c = Math.cos(T.yaw), s = Math.sin(T.yaw);
  return {
    x: T.x + (px * c - py * s),
    y: T.y + (px * s + py * c),
  };
};

const ensureLaserShape = (canvas) => {
  if (window.NAV2D.laser.shape) return;

  // One shape for all points (fast)
  const shape = new window.createjs.Shape();
  shape.name = "laser_scan_overlay";
  canvas.addChild(shape);
  window.NAV2D.laser.shape = shape;
};

const getScene = () => window.NAV2D?.canvas?.scene;

const laserClear = () => {
  const scene = getScene();
  if (!scene) return;

  (window.NAV2D.laser.markers || []).forEach((m) => {
    try { scene.removeChild(m); } catch (e) { }
  });
  window.NAV2D.laser.markers = [];
};

const laserRedraw = () => {
  const scene = getScene();
  if (!scene) return;

  const pts = window.NAV2D.laser.lastPoints || [];
  if (!pts.length) return;

  laserClear();

  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];

    const dot = new window.ROS2D.NavigationArrow({
      size: 18.0,
      strokeSize: 0.0,
      fillColor: window.createjs.Graphics.getRGB(0, 255, 255, 1.0), // cyan
      pulse: false,
    });

    dot.compositeOperation = "lighter";
    dot.x = p.x;
    dot.y = -p.y;
    dot.scaleX = 1.0 / (scene.scaleX || 1.0);
    dot.scaleY = 1.0 / (scene.scaleY || 1.0);

    window.NAV2D.laser.markers.push(dot);
    scene.addChild(dot);
  }
};

// expose so you can call on tab switch
window.NAV2D.laser = window.NAV2D.laser || {};
window.NAV2D.laser.redraw = laserRedraw;
window.NAV2D.laser.clear = laserClear;

const drawLaserPoints = (canvas) => {
  const shape = window.NAV2D?.laser?.shape;
  if (!shape) return;

  const pts = window.NAV2D.laser.lastPoints || [];
  const g = shape.graphics;
  g.clear();

  if (!pts.length) return;

  // ✅ robust color (avoids black rgba bug)
  g.beginFill(window.createjs.Graphics.getRGB(255, 0, 0, 0.9));

  // ✅ dynamic radius: keep visible at any zoom
  // base radius in meters, but scale up when zoomed out
  const zoom = canvas.scaleX || 1.0;
  const r = Math.max(0.04, window.NAV2D.laser.dotRadius / zoom); // good visibility

  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    g.drawCircle(p.x, -p.y, r);
  }

  g.endFill();
};

const initLaserScanOverlay = (ros) => {
  console.log("✅ [LaserOverlay] init called");

  if (!ros || !window.NAV2D?.canvas?.scene) {
    console.log("❌ [LaserOverlay] ros/canvas missing");
    return;
  }

  // IMPORTANT: never freeze scene reference (it changes on tab switch)
  const getCanvas = () => window.NAV2D?.canvas?.scene;

  // =========================
  // CONFIG
  // =========================
  const mapFrame = (window.AppConfig?.NAV2_MAP_FRAME || "map").replace(/^\/+/, "");
  const odomFrame = (window.AppConfig?.ROBOT_POSE_FRAME || "odom").replace(/^\/+/, "");
  const baseLink = (window.AppConfig?.ROBOT_BASE_FRAME || "ebot_base_link").replace(/^\/+/, "");
  const baseMid = (window.NAV2D?.laser?.intermediateBase || "").replace(/^\/+/, "");
  // const baseMid = (window.NAV2D?.laser?.intermediateBase || "ebot_base").replace(/^\/+/, "");
  const scanTopic = (window.AppConfig?.LASER_SCAN_TOPIC || "/scan");

  const MAX_POINTS = 600; // more visible
  const DRAW_EVERY_N = 2;

  // =========================
  // STATE
  // =========================
  window.NAV2D.laser = window.NAV2D.laser || {};
  window.NAV2D.laser.enabled = true;
  window.NAV2D.laser.markers = window.NAV2D.laser.markers || [];
  window.NAV2D.laser.lastPoints = window.NAV2D.laser.lastPoints || [];

  // TF store
  const TF = new Map();
  const norm = (s) => (s || "").replace(/^\/+/, "");
  const key = (p, c) => `${norm(p)}->${norm(c)}`;
  const storeRawTF = (parent, child, tf) => { if (parent && child && tf) TF.set(key(parent, child), tf); };
  const getRawTF = (parent, child) => TF.get(key(parent, child)) || null;

  // =========================
  // MATH (2D)
  // =========================
  const quatToYaw = (q) => {
    if (!q) return 0.0;
    const siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny_cosp, cosy_cosp);
  };

  const rawTo2D = (tf) => {
    if (!tf?.translation || !tf?.rotation) return null;
    return { x: tf.translation.x ?? 0, y: tf.translation.y ?? 0, theta: quatToYaw(tf.rotation) };
  };

  const invert2D = (T) => {
    const c = Math.cos(T.theta), s = Math.sin(T.theta);
    const ix = -(c * T.x + s * T.y);
    const iy = -(-s * T.x + c * T.y);
    return { x: ix, y: iy, theta: -T.theta };
  };

  const compose2D = (A, B) => {
    const c = Math.cos(A.theta), s = Math.sin(A.theta);
    return { x: A.x + (B.x * c - B.y * s), y: A.y + (B.x * s + B.y * c), theta: A.theta + B.theta };
  };

  const transformPoint2D = (T, x, y) => {
    const c = Math.cos(T.theta), s = Math.sin(T.theta);
    return { x: T.x + x * c - y * s, y: T.y + x * s + y * c };
  };

  const get2D = (parent, child) => {
    const fwd = getRawTF(parent, child);
    if (fwd) return rawTo2D(fwd);

    const rev = getRawTF(child, parent);
    if (rev) {
      const T = rawTo2D(rev);
      return T ? invert2D(T) : null;
    }
    return null;
  };

  const getChain2D = (frames) => {
    let T = { x: 0, y: 0, theta: 0 };
    for (let i = 0; i < frames.length - 1; i++) {
      const Tab = get2D(frames[i], frames[i + 1]);
      if (!Tab) return null;
      T = compose2D(T, Tab);
    }
    return T;
  };

  // =========================
  // TF SUBS (/tf + /tf_static)
  // =========================
  const subTF = (topicName) => {
    const t = new window.ROSLIB.Topic({ ros, name: topicName, messageType: "tf2_msgs/TFMessage" });
    t.subscribe((msg) => {
      (msg.transforms || []).forEach((tr) => storeRawTF(tr.header.frame_id, tr.child_frame_id, tr.transform));
    });
    return t;
  };

  if (!window.NAV2D.laser._tfSubscribed) {
    window.NAV2D.laser._tfSubscribed = true;
    subTF("/tf");
    subTF("/tf_static");
  }

  // =========================
  // DRAW (use current scene!)
  // =========================
  const clearLaserMarkers = () => {
    const canvas = getCanvas();
    if (!canvas) return;
    window.NAV2D.laser.markers.forEach((m) => { try { canvas.removeChild(m); } catch (e) { } });
    window.NAV2D.laser.markers = [];
  };

  const makeDot = (canvas, x, y) => {
    // ✅ BIG + bright + no stroke (no black)
    const dot = new window.ROS2D.NavigationArrow({
      size: 18.0,
      strokeSize: 0.0,
      fillColor: window.createjs.Graphics.getRGB(0, 255, 255, 1.0), // CYAN
      pulse: false,
    });

    dot.compositeOperation = "lighter";
    dot.alpha = 1.0;

    dot.x = x;
    dot.y = -y;

    // keep same size while zooming
    dot.scaleX = 1.0 / (canvas.scaleX || 1.0);
    dot.scaleY = 1.0 / (canvas.scaleY || 1.0);

    return dot;
  };

  // ✅ expose redraw for tab switching
  window.NAV2D.laser.redraw = () => {
    const canvas = getCanvas();
    if (!canvas) return;
    const pts = window.NAV2D.laser.lastPoints || [];
    if (!pts.length) return;

    clearLaserMarkers();
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      const dot = makeDot(canvas, p.x, p.y);
      window.NAV2D.laser.markers.push(dot);
      canvas.addChild(dot);
    }
  };

  // =========================
  // LASER SUB
  // =========================
  if (window.NAV2D.laser.scan) {
    try { window.NAV2D.laser.scan.unsubscribe(); } catch (e) { }
    window.NAV2D.laser.scan = null;
  }

  const scan = new window.ROSLIB.Topic({
    ros,
    name: scanTopic,
    messageType: "sensor_msgs/msg/LaserScan",
    throttle_rate: 80,
  });

  let rx = 0;

  scan.subscribe((msg) => {
    rx++;
    if (!window.NAV2D.laser.enabled) return;
    if (rx % DRAW_EVERY_N !== 0) return;

    const canvas = getCanvas();
    if (!canvas) return;

    const laserFrame = norm(msg?.header?.frame_id);
    if (!laserFrame) return;

    const chain1 = [mapFrame, odomFrame, baseLink, baseMid, laserFrame];
    const chain2 = [mapFrame, odomFrame, baseLink, laserFrame];
    const T_map_laser = getChain2D(chain1) || getChain2D(chain2);
    if (!T_map_laser) return;

    const ranges = msg.ranges || [];
    const angleMin = msg.angle_min ?? 0;
    const angleInc = msg.angle_increment ?? 0;
    const rMin = msg.range_min ?? 0.05;
    const rMax = msg.range_max ?? 30.0;

    const pts = [];
    const step = Math.max(1, Math.floor(ranges.length / MAX_POINTS));

    for (let i = 0; i < ranges.length && pts.length < MAX_POINTS; i += step) {
      const r = ranges[i];
      if (!Number.isFinite(r)) continue;
      if (r < rMin || r > rMax) continue;

      const a = angleMin + i * angleInc;
      const lx = r * Math.cos(a);
      const ly = r * Math.sin(a);

      pts.push(transformPoint2D(T_map_laser, lx, ly));
    }

    window.NAV2D.laser.lastPoints = pts;
    window.NAV2D.laser.redraw();


    // draw to CURRENT canvas
    // window.NAV2D.laser.redraw();
  });

  window.NAV2D.laser.scan = scan;

  console.log("✅ [LaserOverlay] init complete", { mapFrame, odomFrame, baseLink, baseMid, scanTopic });
};





const initLaserScanOverlay_ScanOnly = (ros) => {
  console.log("✅ [ScanOnly] initLaserScanOverlay_ScanOnly called");

  if (!ros || !window.NAV2D?.canvas?.scene) {
    console.log("❌ [ScanOnly] ros/canvas missing", {
      ros: !!ros,
      canvas: !!window.NAV2D?.canvas,
      scene: !!window.NAV2D?.canvas?.scene,
    });
    return;
  }

  const canvas = window.NAV2D.canvas.scene;

  // Create shape once
  window.NAV2D.scanOnly = window.NAV2D.scanOnly || {};
  if (!window.NAV2D.scanOnly.shape) {
    window.NAV2D.scanOnly.shape = new window.createjs.Shape();
    canvas.addChild(window.NAV2D.scanOnly.shape);
    console.log("✅ [ScanOnly] shape created and added to canvas");
  } else {
    console.log("✅ [ScanOnly] shape already exists");
  }

  const topicName = window.AppConfig?.LASER_SCAN_TOPIC || "/scan";
  console.log("✅ [ScanOnly] subscribing LaserScan", {
    topic: topicName,
    type: "sensor_msgs/msg/LaserScan",
  });

  const scanTopic = new window.ROSLIB.Topic({
    ros,
    name: topicName,
    messageType: "sensor_msgs/msg/LaserScan",
    throttle_rate: 100, // ms
  });

  let rx = 0;

  scanTopic.subscribe((scan) => {
    rx++;

    const ranges = scan?.ranges || [];
    const angleMin = scan?.angle_min ?? 0;
    const angleInc = scan?.angle_increment ?? 0;
    const rMin = scan?.range_min ?? 0;
    const rMax = scan?.range_max ?? Infinity;

    // Count validity
    let finiteCount = 0;
    let inRangeCount = 0;
    let minR = Infinity;
    let maxR = -Infinity;
    let zeroCount = 0;

    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (r === 0) zeroCount++;
      if (Number.isFinite(r)) {
        finiteCount++;
        if (r < minR) minR = r;
        if (r > maxR) maxR = r;
        if (r >= rMin && r <= rMax) inRangeCount++;
      }
    }

    if (rx === 1 || rx % 20 === 0) {
      console.log("✅ [ScanOnly] stats", {
        rx,
        frame_id: scan?.header?.frame_id,
        ranges_len: ranges.length,
        finiteCount,
        inRangeCount,
        zeroCount,
        minR: Number.isFinite(minR) ? minR : null,
        maxR: Number.isFinite(maxR) ? maxR : null,
        range_min: rMin,
        range_max: rMax,
      });
    }

    // Draw
    const g = window.NAV2D.scanOnly.shape.graphics;
    g.clear();
    g.beginFill(window.createjs.Graphics.getRGB(255, 0, 0, 0.9));

    let drawn = 0;

    // DEBUG DRAW RULE:
    // - only skip NaN/Inf and r<=0
    // - do NOT filter using range_min/range_max (so you can see *something*)
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!Number.isFinite(r)) continue;
      if (r <= 0) continue;

      const a = angleMin + i * angleInc;
      const x = r * Math.cos(a);
      const y = r * Math.sin(a);

      // Bigger dot so it is visible
      g.drawCircle(x, -y, 0.08);
      drawn++;
      if (drawn > 2500) break;
    }

    g.endFill();

    // keep same size while zooming
    window.NAV2D.scanOnly.shape.scaleX = 1.0 / canvas.scaleX;
    window.NAV2D.scanOnly.shape.scaleY = 1.0 / canvas.scaleY;

    if (rx === 1 || rx % 20 === 0) {
      console.log("✅ [ScanOnly] drawn", { drawn });
    }
  });

  console.log("✅ [ScanOnly] subscription active");
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
      const robotMarker = window.NAV2D.ensureRobotMarker();
      if (!robotMarker) return;

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
      // window.NAV2D.mapInited = false;
    },
  );

  // /* ROBOT MARKER SECTION: */
  // const robotMarker = createCanvasPoint(25, {
  //   r: 0,
  //   g: 0,
  //   b: 255,
  //   a: 1,
  // });
  // robotMarker.visible = false;
  // canvas.addChild(robotMarker);

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

    const endPos = canvas.globalToRos(event.stageX, event.stageY);
    const endVec = new window.ROSLIB.Vector3(endPos);

    const xDelta = endVec.x - positionVectorItem.x;
    const yDelta = endVec.y - positionVectorItem.y;

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

  if (!pointDetails || !pointDetails.position || !pointDetails.orientation) {
    console.log("❌ [sendPointToRobot] No waypoint selected", {
      finishedPointItem: window.NAV2D.finishedPointItem,
      orientatedPointItem: window.NAV2D.orientatedPointItem,
    });

    // optional: publish message to UI topic
    try {
      const messageTopic = new window.ROSLIB.Topic({
        ros,
        name: window.AppConfig.UI_MESSAGE_TOPIC,
        messageType: "std_msgs/String",
      });
      messageTopic.publish(new window.ROSLIB.Message({
        data: "⚠️ Please set a waypoint on the map before sending.",
      }));
    } catch (e) { }

    return; // ✅ prevent crash
  }

  const { hours, minutes } = time || {};

  const wayPoint = new window.ROSLIB.Topic({
    ros,
    name: "/new_way_point",
    messageType: "geometry_msgs/PoseWithCovarianceStamped",
  });

  const covariance = new Array(36).fill(0.0);
  covariance[0] = 3;
  covariance[1] = Number(hours) || 0;
  covariance[2] = Number(minutes) || 0;

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
          x: pointDetails.orientation.x || 0.0,
          y: pointDetails.orientation.y || 0.0,
          z: pointDetails.orientation.z || 0.0,
          w: pointDetails.orientation.w || 1.0,
        },
      },
      covariance,
    },
  };

  console.log("✅ [sendPointToRobot] Publishing waypoint:", messageObject);

  wayPoint.publish(new window.ROSLIB.Message(messageObject));

  // ✅ clear only after successful publish
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