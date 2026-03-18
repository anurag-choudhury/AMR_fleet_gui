import React, { useContext, useState, useRef, useEffect } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { AppConfig } from "../shared/constants/index";
import { testStructure } from "../shared/constants/testjson";

import { RosContext } from "../app/App";

import Map from "../components/Map";
import Logs from "../components/Logs";
import Joystick from "../components/Joystick";
import FilesModal from "../components/modal/FilesModal";
import Button from "../shared/ui/Button";

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

const MapPage = () => {
  const ros = useContext(RosContext);
  const [openModal, setOpenModal] = useState(false);
  const [filesData, setFilesData] = useState([]);
  const [selectedFile, setSelectedFile] = useState({ group: "", map: "" });
  const [inEditMode, setIsEditMode] = useState(false);
  const [isPoseMode, setIsPoseMode] = useState(false);

  // Sync React state when nav2d.js resets isInitialPoseMode after pose is published
  useEffect(() => {
    const interval = setInterval(() => {
      if (isPoseMode && window.isInitialPoseMode === false) {
        setIsPoseMode(false);
      }
    }, 200);
    return () => clearInterval(interval);
  }, [isPoseMode]);

  const modalKey = useRef(null);
  const filesModalType = useRef(null);
  const isFilesModalWithInput = useRef(null);
  const filesModalHeader = useRef(null);
  const filesModalPlaceholder = useRef(null);

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
    getMockedData();

    const currentFilesResponseTopic = filesResonseTopic.current;

    currentFilesResponseTopic.subscribe((data) => {
      const response = data.data;
      const responseObject = JSON.parse(response);
      const serializedArray = removeCsv(responseObject.structure);
      const arrayWithSpaces =
        replaceUnderscoresInKeysAndValues(serializedArray);
      const activeFilesWithSpacec = replaceUnderscoresInKeysAndValues(
        responseObject.active_files,
      );

      setFilesData(arrayWithSpaces);
      setSelectedFile(activeFilesWithSpacec);
    });

    filesReqTopic.current.publish();

    return () => currentFilesResponseTopic.unsubscribe();
  }, []);

  /* BUTTON HANDLERS */
  const onNewMapClick = () => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: "build_map" }),
    );
    setIsEditMode(true);
  };

  const onSaveMapClick = () => {
    modalKey.current = "SaveMap";
    filesModalType.current = "selectGroup";
    isFilesModalWithInput.current = true;
    filesModalHeader.current = "Select group for saving the map";
    filesModalPlaceholder.current = "Enter map name...";
    setOpenModal(true);
  };

  const onCreateGroupClick = () => {
    modalKey.current = "CreateGroup";
    filesModalType.current = "selectGroup";
    isFilesModalWithInput.current = true;
    filesModalHeader.current = "Create group";
    filesModalPlaceholder.current = "Enter new group name...";
    setOpenModal(true);
  };

  const onChangeMapClick = () => {
    modalKey.current = "ChangeMap";
    filesModalType.current = "selectMap";
    isFilesModalWithInput.current = false;
    filesModalHeader.current = "Choose map:";
    setOpenModal(true);
  };

  const onRenameMapClick = () => {
    modalKey.current = "RenameMap";
    filesModalType.current = "selectMap";
    isFilesModalWithInput.current = true;
    filesModalHeader.current = "Choose map you want to rename";
    filesModalPlaceholder.current = "Enter map new name...";
    setOpenModal(true);
  };

  const onDeleteMapClick = () => {
    modalKey.current = "DeleteMap";
    filesModalType.current = "selectMap";
    isFilesModalWithInput.current = false;
    filesModalHeader.current = "Choose map you want to delete";
    setOpenModal(true);
  };

  const onDeleteGroupClick = () => {
    modalKey.current = "DeleteGroup";
    filesModalType.current = "selectGroup";
    isFilesModalWithInput.current = false;
    filesModalHeader.current = "Choose group you want to delete";
    setOpenModal(true);
  };

  /* FORM SUBMIT HANDLER */
  const onFormSubmitHandler = (data) => {
    setOpenModal(false);
    let isBreaked = false;

    if (data) {
      const operationsConfig = {
        SaveMap: {
          path: "save_map",
          data: { group: data.group, map: data.inputValue },
          preActions: () => {
            if (!data.group || !data.inputValue) {
              isBreaked = true;
              toast.warn("You need to select group and provide map name");
              return;
            }
            if (data.inputValue.toString().includes("_")) {
              isBreaked = true;
              toast.warn("Symbol '_' is forbidden");
              return;
            }
          },
        },
        CreateGroup: {
          path: "create_group",
          data: { group: data.inputValue },
          preActions: () => {
            if (!data.inputValue) {
              isBreaked = true;
              toast.warn("You need to provide group name");
              return;
            }
            if (data.inputValue.toString().includes("_")) {
              isBreaked = true;
              toast.warn("Symbol '_' is forbidden");
              return;
            }
          },
        },
        ChangeMap: {
          path: "change_map",
          data: { group: data.group, map: data.map },
        },
        RenameMap: {
          path: "rename_map",
          data: {
            group: data.group,
            map_old: data.map,
            map_new: data.inputValue,
          },
          preActions: () => {
            if (!data.map || !data.inputValue) {
              isBreaked = true;
              toast.warn("You need to choose map and provide map new name");
              return;
            }
            if (data.inputValue.toString().includes("_")) {
              isBreaked = true;
              toast.warn("Symbol '_' is forbidden");
              return;
            }
          },
        },
        DeleteMap: {
          path: "delete_map",
          data: { group: data.group, map: data.map },
        },
        DeleteGroup: {
          path: "delete_group",
          data: { group: data.group },
        },
      };

      const currentOperation = operationsConfig[modalKey.current];

      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();
        if (isBreaked) return;

        const objectWithoutSpaces = processObjectStrings(currentOperation.data);
        const stringifiedObjToSend = JSON.stringify(objectWithoutSpaces);
        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;

        uiOperationTopic.current.publish(
          new window.ROSLIB.Message({ data: messageToSend }),
        );

        currentOperation.postActions && currentOperation.postActions();
      }
      setIsEditMode(false);
    }

    modalKey.current = null;
    filesModalType.current = null;
    isFilesModalWithInput.current = null;
    filesModalHeader.current = null;
    filesModalPlaceholder.current = null;
  };

  // Ensure window flag is initialised only once
  if (window.isInitialPoseMode === undefined) {
    window.isInitialPoseMode = false;
  }

  const init_pose = () => {
    window.isInitialPoseMode = true;
    setIsPoseMode(true);
    console.log("Click and drag on the map to set the initial pose of the robot.");
  };

  const cancel_pose = () => {
    window.isInitialPoseMode = false;
    setIsPoseMode(false);
  };

  return (
    <>
      <ToastContainer />

      {openModal && (
        <FilesModal
          filesList={filesData}
          headerText={filesModalHeader.current}
          mode={filesModalType.current}
          hasInput={isFilesModalWithInput.current}
          inputPlaceholder={filesModalPlaceholder.current}
          modalHandler={onFormSubmitHandler}
        />
      )}

      {/* Main Wrapper: Full viewport height minus the parent header */}
      <div className="flex flex-col h-[calc(100vh-60px)] w-full bg-slate-950 overflow-hidden">

        {/* TOP: Status Bar */}
        <div className="flex items-center justify-between px-6 py-4 bg-slate-900 border-b border-slate-800 flex-shrink-0">
          <div className="flex items-center gap-6">
            <h1 className="text-xl font-bold text-white tracking-wide">AMR Controller</h1>
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
          </div>

          <div className="flex items-center gap-3">
            {isPoseMode && (
              <div className="flex items-center gap-2 bg-amber-500/20 border border-amber-500/40 rounded-md px-3 py-1.5">
                <span className="relative flex h-2 w-2 flex-shrink-0">
                  <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-amber-400 opacity-75"></span>
                  <span className="relative inline-flex rounded-full h-2 w-2 bg-amber-500"></span>
                </span>
                <span className="text-xs font-medium text-amber-300">
                  Pose mode — click &amp; drag on map
                </span>
              </div>
            )}
            {isPoseMode ? (
              <button
                onClick={cancel_pose}
                className="rounded-lg bg-gradient-to-r from-red-600 to-red-700 hover:from-red-700 hover:to-red-800 text-white font-medium py-2 px-6 transition-all duration-200 shadow-sm hover:shadow-md text-sm flex items-center gap-2"
              >
                Cancel Pose
              </button>
            ) : (
              <button
                onClick={init_pose}
                className="rounded-lg bg-gradient-to-r from-blue-600 to-blue-700 hover:from-blue-700 hover:to-blue-800 text-white font-medium py-2 px-6 transition-all duration-200 shadow-sm hover:shadow-md text-sm"
              >
                Initialize Pose
              </button>
            )}
          </div>
        </div>

        {/* MIDDLE: Primary Map Area */}
        <div className="flex-1 min-h-0 relative bg-slate-950 p-2">
          {/* Map wrapper to ensure it fills the container */}
          <div className="w-full h-full rounded-md overflow-hidden bg-white shadow-inner flex items-center justify-center">
            <Map />
          </div>
        </div>

        {/* BOTTOM: Split Panel */}
        <div className="h-[40vh] min-h-[300px] flex-shrink-0 bg-slate-900 flex flex-row w-full border-t border-slate-800">

          {/* 1. Messages Panel */}
          <div className="flex-1 flex flex-col min-w-0 border-r border-slate-800 p-4">
            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-2">Messages</span>
            <div className="flex-1 bg-slate-950 rounded-md p-3 overflow-auto border border-slate-800 text-sm">
              <Logs />
            </div>
          </div>

          {/* 2. Joystick Container - REDUCED WIDTH & CENTERED JOYSTICK */}
          <div className="w-60 flex-shrink-0 flex flex-col items-center border-r border-slate-800 p-4 bg-slate-900">
            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-2 w-full text-center">Controller</span>
            <div className="flex-1 w-full flex items-center justify-center relative">
              {/* Removed the CSS scale transform that was misaligning the joystick */}
              <Joystick />
            </div>
          </div>

          {/* 3. Management Panels */}
          <div className="w-[450px] flex-shrink-0 flex flex-col p-4 overflow-y-auto">

            {/* Map Management Block */}
            <div className="mb-4">
              <span className="text-xs font-bold text-slate-400 uppercase tracking-wider block mb-3">Map Management</span>
              <div className="grid grid-cols-3 gap-2">
                <Button size="small" onBtnClick={onChangeMapClick} type={inEditMode ? "disabled" : ""}>
                  Change
                </Button>
                <Button size="small" onBtnClick={onSaveMapClick} type={inEditMode ? "" : "disabled"}>
                  Save
                </Button>
                <Button size="small" onBtnClick={onNewMapClick} type={inEditMode ? "disabled" : ""}>
                  Create
                </Button>
                <Button size="small" onBtnClick={onRenameMapClick} type={inEditMode ? "disabled" : ""}>
                  Rename
                </Button>
                <Button size="small" onBtnClick={onDeleteMapClick} type={inEditMode ? "disabled" : ""}>
                  Delete
                </Button>
              </div>
            </div>

            <div className="h-px bg-slate-800 w-full mb-4"></div>

            {/* Group Management Block */}
            <div>
              <span className="text-xs font-bold text-slate-400 uppercase tracking-wider block mb-3">Group Management</span>
              <div className="grid grid-cols-2 gap-2 w-2/3">
                <Button size="small" onBtnClick={onCreateGroupClick}>
                  Create
                </Button>
                <Button size="small" onBtnClick={onDeleteGroupClick} type={inEditMode ? "disabled" : ""}>
                  Delete
                </Button>
              </div>
            </div>

          </div>
        </div>

      </div>
    </>
  );
};

export default MapPage;