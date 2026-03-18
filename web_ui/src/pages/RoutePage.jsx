import React, {
  useContext,
  useRef,
  useState,
  useEffect,
  useCallback,
} from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";
import { testStructure } from "../shared/constants/testjson";

import TimeModal from "../components/modal/TimeModal";
import FilesModal from "../components/modal/FilesModal";
import TextInputModal from "../components/modal/TextInputModal";

import Map from "../components/Map";
import Logs from "../components/Logs";
import Button from "../shared/ui/Button";

const removeCsv = (data) => {
  if (Array.isArray(data)) {
    return data.map((item) => {
      if (typeof item === "string") {
        return item.replace(".csv", "");
      }
      return removeCsv(item);
    });
  } else if (typeof data === "object" && data !== null) {
    const result = {};
    for (const key in data) {
      result[key] = removeCsv(data[key]);
    }
    return result;
  }
  return data;
};

function replaceUnderscoresInKeysAndValues(obj) {
  if (Array.isArray(obj)) {
    return obj.map((item) => replaceUnderscoresInKeysAndValues(item));
  } else if (typeof obj === "object" && obj !== null) {
    const newObj = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        const newKey = key.replace(/_/g, " ");
        newObj[newKey] = replaceUnderscoresInKeysAndValues(obj[key]);
      }
    }
    return newObj;
  } else if (typeof obj === "string") {
    return obj.replace(/_/g, " ");
  }
  return obj;
}

const processObjectStrings = (obj) => {
  if (typeof obj === "object" && obj !== null) {
    const result = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        result[key] = processObjectStrings(obj[key]);
      }
    }
    return result;
  } else if (typeof obj === "string") {
    return obj.trim().replace(/\s/g, "_");
  }
  return obj;
};

const findMapArray = (structure, activeFiles) => {
  const groupObject = structure.find((floor) =>
    Object.prototype.hasOwnProperty.call(floor, activeFiles.group),
  );

  if (groupObject) {
    const mapsArray = groupObject[activeFiles.group];

    const mapObject = mapsArray.find((room) =>
      Object.prototype.hasOwnProperty.call(room, activeFiles.map),
    );

    if (mapObject) {
      return mapObject[activeFiles.map];
    }
  }

  return [];
};

const removePointFromCanvas = () => {
  const markerOnMap = window.NAV2D.orientatedPointItem;
  window.NAV2D.pointsArray = window.NAV2D.pointsArray.filter(
    (marker) => marker !== markerOnMap,
  );
  window.NAV2D.canvas.scene.removeChild(markerOnMap);
  window.NAV2D.finishedPointItem = null;
  window.NAV2D.orientatedPointItem = null;
};

const RoutePage = () => {
  const ros = useContext(RosContext);
  // eslint-disable-next-line no-unused-vars
  const [selectedPointType, setSelectedPointType] = useState(null);
  const [pointsSettable, setPointsSettable] = useState(false);
  // eslint-disable-next-line no-unused-vars
  const [hoursValue, setHoursValue] = useState("0");
  const latestHoursValue = useRef(hoursValue);
  // eslint-disable-next-line no-unused-vars
  const [minutesValue, setMinutesValue] = useState("0");
  const latestMinutesValue = useRef(minutesValue);

  useEffect(() => {
    latestHoursValue.current = hoursValue;
    latestMinutesValue.current = minutesValue;
  }, [hoursValue, minutesValue]);

  const [selectedFile, setSelectedFile] = useState({
    group: "",
    map: "",
    route: "",
  });
  const [openRouteModal, setOpenRouteModal] = useState(false);
  const [openTimeModal, setOpenTimeModal] = useState(false);
  const [openInputModal, setOpenInputModal] = useState(false);

  const [filesData, setFilesData] = useState([]);
  const [fullFilesData, setFullFilesData] = useState([]);

  const childRef = useRef(null);
  const routesModalType = useRef(null);
  const isRoutesModalWithInput = useRef(null);
  const routesModalHeader = useRef(null);
  const modalKey = useRef(null);

  const textInputHeader = useRef(null);
  const textInputPlaceholder = useRef(null);

  /* TOPICS */
  const filesReqTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: "/nav_data_req",
      messageType: "std_msgs/Empty",
    }),
  );

  const filesResonseTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: "/nav_data_resp",
      messageType: "std_msgs/String",
    }),
  );

  const uiOperationTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const onMapClickHandler = useCallback(() => {
    if (!window.NAV2D.arePointsSettable) return;

    let timeObj = {
      hours: latestHoursValue.current,
      minutes: latestMinutesValue.current,
    };

    setTimeout(() => {
      if (!timeObj.hours && !timeObj.minutes) {
        toast.warn("Enter hours and minutes");
        removePointFromCanvas();
        return;
      }

      if (timeObj.hours < 0 || timeObj.hours > 23) {
        toast.warn("Hours can't be less then 0 and more then 23");
        removePointFromCanvas();
        return;
      }

      if (timeObj.minutes < 0 || timeObj.minutes > 59) {
        toast.warn("Minutes can't be less then 0 and more then 59");
        removePointFromCanvas();
        return;
      }

      window.NAV2D.sendPointToRobot(ros, timeObj);
    }, 0);
  }, [ros]);

  // eslint-disable-next-line no-unused-vars
  const getMockedData = () => {
    const serializedTestArray = removeCsv(testStructure.structure);
    const serializedTestArrayWithSpaces =
      replaceUnderscoresInKeysAndValues(serializedTestArray);
    const testActiveFileWithSpaces = replaceUnderscoresInKeysAndValues(
      testStructure.active_files,
    );

    setFilesData(serializedTestArrayWithSpaces);
    setSelectedFile(testActiveFileWithSpaces);
  };

  useEffect(() => {
    window.NAV2D.ClearMap();
    onOperationTopicPublish("clear_route");

    getMockedData();

    const currentFilesResponseTopic = filesResonseTopic.current;
    const currentMap = childRef.current;

    currentFilesResponseTopic.subscribe((data) => {
      const response = data.data;
      const responseObject = JSON.parse(response);

      const serializedArray = removeCsv(responseObject.structure);
      const arrayWithSpaces =
        replaceUnderscoresInKeysAndValues(serializedArray);

      const activeFilesWithSpaces = replaceUnderscoresInKeysAndValues({
        group: responseObject.active_files.group,
        map: responseObject.active_files.map,
        route: responseObject.active_files.route.replace(".csv", ""),
      });

      const filtredArrayBySelectedRoute = findMapArray(
        arrayWithSpaces,
        activeFilesWithSpaces,
      );

      setFullFilesData(arrayWithSpaces);
      setFilesData(filtredArrayBySelectedRoute);
      setSelectedFile(activeFilesWithSpaces);
    });

    filesReqTopic.current.publish();

    return () => {
      currentFilesResponseTopic.unsubscribe();
      window.NAV2D.arePointsSettable = false;

      const mapElement = currentMap.getMapRef();
      if (mapElement) {
        mapElement.removeEventListener("mouseup", onMapClickHandler);
        mapElement.removeEventListener("touchend", onMapClickHandler);
      }
    };
  }, [onMapClickHandler]);

  const onOperationTopicPublish = (message) => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: message }),
    );
  };

  /* FROM HANDLERS */
  const onRouteFormSubmitHandler = (data) => {
    setOpenRouteModal(false);

    if (data) {
      const operationsConfig = {
        CHANGE_ROUTE: {
          path: "change_route",
          data: { group: data.group, map: data.map, route: data.route },
          preActions: () => window.NAV2D.ClearMap(),
          postActions: () =>
            setSelectedFile({ group: data.group, map: data.map, route: data.route }),
        },
      };

      const currentOperation = operationsConfig[modalKey.current];
      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();
        const objToSendWithoutSpaces = processObjectStrings(currentOperation.data);
        const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        onOperationTopicPublish(messageToSend);
        currentOperation.postActions && currentOperation.postActions();
      }
    }

    modalKey.current = null;
    routesModalType.current = null;
    isRoutesModalWithInput.current = null;
    routesModalHeader.current = null;
  };

  const onTimeFormSubmitHandler = (data) => {
    setOpenTimeModal(false);
    if (data) {
      window.NAV2D.sendPointToRobot(ros, data);
    } else {
      const markerOnMap = window.NAV2D.orientatedPointItem;
      window.NAV2D.pointsArray = window.NAV2D.pointsArray.filter(
        (marker) => marker !== markerOnMap,
      );
      window.NAV2D.canvas.scene.removeChild(markerOnMap);
      window.NAV2D.finishedPointItem = null;
      window.NAV2D.orientatedPointItem = null;
    }
  };

  const onInputFormSubmitHandler = (data) => {
    setOpenInputModal(false);

    if (data) {
      const operationsConfig = {
        SAVE_ROUTE: {
          path: "save_route",
          data: { group: selectedFile.group, map: selectedFile.map, route: data },
          postActions: () => {
            removeMapListeners();
            setPointsSettable(false);
            selectedFile.route = data;
            setSelectedPointType(null);
            window.NAV2D.pointType = null;
            window.NAV2D.arePointsSettable = false;
          },
        },
        RENAME_ROUTE: {
          path: "rename_route",
          data: { group: selectedFile.group, map: selectedFile.map, route_old: selectedFile.route, route_new: data },
          postActions: () => {
            setSelectedFile({ group: selectedFile.group, map: selectedFile.map, route: data });
          },
        },
      };

      const currentOperation = operationsConfig[modalKey.current];
      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();
        const objToSendWithoutSpaces = processObjectStrings(currentOperation.data);
        const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        onOperationTopicPublish(messageToSend);
        currentOperation.postActions && currentOperation.postActions();
      }
    }

    modalKey.current = null;
    textInputHeader.current = null;
    textInputPlaceholder.current = null;
  };

  /* BUTTON HANDLERS */
  const addMapListeners = () => {
    const mapElement = childRef.current.getMapRef();
    if (mapElement) {
      mapElement.addEventListener("mouseup", onMapClickHandler);
      mapElement.addEventListener("touchend", onMapClickHandler);
    }
  };

  const removeMapListeners = () => {
    const mapElement = childRef.current.getMapRef();
    if (mapElement) {
      mapElement.removeEventListener("mouseup", onMapClickHandler);
      mapElement.removeEventListener("touchend", onMapClickHandler);
    }
  };

  const onNewRouteClick = () => {
    setSelectedPointType(null);
    setSelectedFile({ group: selectedFile.group, map: selectedFile.map, route: "New route" });
    window.NAV2D.pointType = null;
    window.NAV2D.arePointsSettable = true;
    addMapListeners();
    setPointsSettable(true);
    window.NAV2D.ClearMap();
    window.NAV2D.pointsFromTopic = [];
    onOperationTopicPublish("clear_route");
  };

  const onSaveRouteClick = () => {
    if (!pointsSettable) return;
    if (selectedFile.route !== "New route") {
      const dataToSend = { group: selectedFile.group, map: selectedFile.map, route: selectedFile.route };
      const objToSendWithoutSpaces = processObjectStrings(dataToSend);
      const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
      const messageToSend = `save_route/${stringifiedObjToSend}`;
      onOperationTopicPublish(messageToSend);
      setSelectedPointType(null);
      window.NAV2D.pointType = null;
      window.NAV2D.arePointsSettable = false;
      removeMapListeners();
      setPointsSettable(false);
      return;
    }
    modalKey.current = "SAVE_ROUTE";
    textInputHeader.current = "Enter new route name";
    textInputPlaceholder.current = "Name...";
    setOpenInputModal(true);
  };

  const onChangeRouteClick = () => {
    window.NAV2D.arePointsSettable = false;
    setPointsSettable(false);
    modalKey.current = "CHANGE_ROUTE";
    routesModalType.current = "selectRoute";
    isRoutesModalWithInput.current = false;
    routesModalHeader.current = "Select route you want to browse";
    setOpenRouteModal(true);
  };

  const onEditRouteClick = () => {
    if (pointsSettable) {
      setPointsSettable(false);
    } else {
      const currentRouteData = { group: selectedFile.group, map: selectedFile.map, route: selectedFile.route };
      const objToSendWithoutSpaces = processObjectStrings(currentRouteData);
      const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
      const messageToSend = `edit_route/${stringifiedObjToSend}`;
      onOperationTopicPublish(messageToSend);
      window.NAV2D.arePointsSettable = true;
      addMapListeners();
      setPointsSettable(true);
    }
  };

  const onClearRouteClick = () => {
    if (!pointsSettable) return;
    window.NAV2D.ClearMap();
    window.NAV2D.pointsFromTopic = [];
    onOperationTopicPublish("clear_route");
  };

  const onUndoRouteClick = () => {
    if (!pointsSettable) return;
    if (window.NAV2D && window.NAV2D.UndoPoint) {
      window.NAV2D.UndoPoint();
    }
  };

  const onDeleteRouteClick = () => {
    window.NAV2D.ClearMap();
    const currentRouteData = { group: selectedFile.group, map: selectedFile.map, route: selectedFile.route };
    const objToSendWithoutSpaces = processObjectStrings(currentRouteData);
    const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
    const messageToSend = `delete_route/${stringifiedObjToSend}`;
    onOperationTopicPublish(messageToSend);
  };

  const onRenameRouteClick = () => {
    modalKey.current = "RENAME_ROUTE";
    textInputHeader.current = "Enter route new name";
    textInputPlaceholder.current = "Name...";
    setOpenInputModal(true);
  };

  // eslint-disable-next-line no-unused-vars
  const onPointClickHandler = (data) => {
    setSelectedPointType(data);
    window.NAV2D.pointType = data;
  };

  return (
    <>
      <ToastContainer />

      {openRouteModal && (
        <FilesModal
          filesList={fullFilesData}
          headerText={routesModalHeader.current}
          mode={routesModalType.current}
          hasInput={isRoutesModalWithInput.current}
          inputPlaceholder={null}
          modalHandler={onRouteFormSubmitHandler}
        />
      )}

      {openTimeModal && <TimeModal modalHandler={onTimeFormSubmitHandler} />}

      {openInputModal && (
        <TextInputModal
          header={textInputHeader.current}
          placeholder={textInputPlaceholder.current}
          routesList={filesData}
          modalHandler={onInputFormSubmitHandler}
        />
      )}

      {/* Main Wrapper: Full viewport height minus the parent header */}
      <div className="flex flex-col h-[calc(100vh-60px)] w-full bg-slate-950 overflow-hidden">
        
        {/* TOP: Status Bar */}
        <div className="flex items-center justify-between px-6 py-4 bg-slate-900 border-b border-slate-800 flex-shrink-0">
          <div className="flex items-center gap-6">
            <h1 className="text-xl font-bold text-white tracking-wide">Route Editor</h1>
            <div className="flex items-center gap-2">
              <span className="inline-block w-3 h-3 rounded-full bg-emerald-500"></span>
              <span className="text-sm text-slate-300">
                Group: <span className="font-semibold text-white">{selectedFile.group || "Null"}</span>
              </span>
            </div>
            <div className="flex items-center gap-2">
              <span className="inline-block w-3 h-3 rounded-full bg-blue-500"></span>
              <span className="text-sm text-slate-300">
                Map: <span className="font-semibold text-white">{selectedFile.map || "Null"}</span>
              </span>
            </div>
            <div className="flex items-center gap-2">
              <span className="inline-block w-3 h-3 rounded-full bg-purple-500"></span>
              <span className="text-sm text-slate-300">
                Route: <span className="font-semibold text-white">{selectedFile.route || "Null"}</span>
              </span>
            </div>
          </div>
          {pointsSettable && (
            <div className="flex items-center gap-2 bg-emerald-500/20 border border-emerald-500/40 rounded-md px-3 py-1.5">
              <span className="relative flex h-2 w-2 flex-shrink-0">
                <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
                <span className="relative inline-flex rounded-full h-2 w-2 bg-emerald-500"></span>
              </span>
              <span className="text-xs font-medium text-emerald-300">
                Edit mode — click &amp; drag to add waypoints
              </span>
            </div>
          )}
        </div>

        {/* MIDDLE: Primary Map Area */}
        <div className="flex-1 min-h-0 relative bg-slate-950 p-2">
          {/* Map wrapper to ensure it fills the container */}
          <div className="w-full h-full rounded-md overflow-hidden bg-white shadow-inner flex items-center justify-center">
             <Map ref={childRef} />
          </div>
        </div>

        {/* BOTTOM: Split Panel (Messages | Route Operations) */}
        <div className="h-[40vh] min-h-[300px] flex-shrink-0 bg-slate-900 flex flex-row w-full border-t border-slate-800">
          
          {/* 1. Messages Panel */}
          <div className="flex-1 flex flex-col min-w-0 border-r border-slate-800 p-4">
            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-2">Messages</span>
            <div className="flex-1 bg-slate-950 rounded-md p-3 overflow-auto border border-slate-800 text-sm">
              <Logs />
            </div>
          </div>

          {/* 2. Route Operations Management Block */}
          <div className="w-[450px] flex-shrink-0 flex flex-col p-4 overflow-y-auto">
            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider block mb-3">Route Operations</span>
            <div className="grid grid-cols-3 gap-2">
              <Button size="small" onBtnClick={onEditRouteClick}>
                {pointsSettable ? "Cancel" : "Edit"}
              </Button>
              <Button size="small" onBtnClick={onChangeRouteClick} type={pointsSettable ? "disabled" : ""}>
                Change
              </Button>
              <Button size="small" onBtnClick={onSaveRouteClick} type={pointsSettable ? "" : "disabled"}>
                Save
              </Button>
              <Button size="small" onBtnClick={onNewRouteClick} type={pointsSettable ? "disabled" : ""}>
                Create
              </Button>
              <Button size="small" onBtnClick={onRenameRouteClick} type={pointsSettable || selectedFile.route === "Null" ? "disabled" : ""}>
                Rename
              </Button>
              <Button size="small" onBtnClick={onDeleteRouteClick} type={pointsSettable ? "disabled" : ""}>
                Delete
              </Button>
              <Button size="small" onBtnClick={onClearRouteClick} type={!pointsSettable ? "disabled" : ""}>
                Clear
              </Button>
              <Button size="small" onBtnClick={onUndoRouteClick} type={!pointsSettable ? "disabled" : ""}>
                Undo
              </Button>
            </div>
          </div>

        </div>

      </div>
    </>
  );
};

export default RoutePage;