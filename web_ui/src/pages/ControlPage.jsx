import React, { useContext, useRef, useState, useEffect } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { AppConfig } from "../shared/constants/index";
import { testStructure } from "../shared/constants/testjson";
import { RosContext } from "../app/App";

import Map from "../components/Map";
import Logs from "../components/Logs";
import FilesModal from "../components/modal/FilesModal";

import Button from "../shared/ui/Button";
import RoundedButton from "../shared/ui/RoundedButton";

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

const ControlPage = () => {
  const ros = useContext(RosContext);
  const [openModal, setOpenModal] = useState(false);
  const [filesData, setFilesData] = useState([]);
  const [selectedFile, setSelectedFile] = useState({ group: "", map: "" });

  const modalKey = useRef(null);
  const filesModalType = useRef(null);
  const isFilesModalWithInput = useRef(null);
  const filesModalHeader = useRef(null);
  const filesModalPlaceholder = useRef(null);

  const uiOperationTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

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

  const uiMessageTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_MESSAGE_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  // eslint-disable-next-line no-unused-vars
  const getMockedData = () => {
    const serializedTestArray = removeCsv(testStructure.structure);
    const serializedTestArrayWithSpaces = replaceUnderscoresInKeysAndValues(serializedTestArray);
    const testActiveFileWithSpaces = replaceUnderscoresInKeysAndValues(testStructure.active_files);

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
      const arrayWithSpaces = replaceUnderscoresInKeysAndValues(serializedArray);
      const activeFilesWithSpacec = replaceUnderscoresInKeysAndValues(responseObject.active_files);

      setFilesData(arrayWithSpaces);
      setSelectedFile(activeFilesWithSpacec);
    });

    filesReqTopic.current.publish();

    return () => currentFilesResponseTopic.unsubscribe();
  }, []);

  // Subscribe to /ui_message for waypoint progress notifications
  useEffect(() => {
    const messageTopic = uiMessageTopic.current;

    messageTopic.subscribe(({ data }) => {
      try {
        const parsed = JSON.parse(data);
        const code = parsed.code || "";
        const details = parsed.details || {};
        const wpNum = details.wp;
        const lap = details.lap;

        switch (code) {
          case "WP_REACHED":
            toast.success(`✅ Waypoint ${wpNum} reached${lap ? ` (Lap ${lap})` : ""}`, {
              position: "top-right",
              autoClose: 3000,
            });
            break;
          case "WP_STARTED":
            toast.info(`🚀 Moving to Waypoint ${wpNum}${lap ? ` (Lap ${lap})` : ""}`, {
              position: "top-right",
              autoClose: 2000,
            });
            break;
          case "HOME_LAP_DONE":
            toast.success(`🏁 Lap ${lap || ""} completed!`, {
              position: "top-right",
              autoClose: 4000,
            });
            break;
          case "HOME_NAV_FAIL":
            toast.error(`❌ Navigation failed at WP ${wpNum}${lap ? ` (Lap ${lap})` : ""}`, {
              position: "top-right",
              autoClose: 5000,
            });
            break;
          case "ROUTE_FINISHED":
            toast.success("🎉 Route completed!", {
              position: "top-right",
              autoClose: 5000,
            });
            break;
          case "NAV_GOAL_REACHED":
            toast.success(`✅ Goal reached${wpNum ? ` — WP ${wpNum}` : ""}`, {
              position: "top-right",
              autoClose: 3000,
            });
            break;
          default:
            // For any other message with a waypoint reference, show info
            if (wpNum !== undefined && code) {
              toast.info(`WP ${wpNum}: ${parsed.message || code}`, {
                position: "top-right",
                autoClose: 3000,
              });
            }
            break;
        }
      } catch (e) {
        // Not JSON or no code field — ignore
      }
    });

    return () => messageTopic.unsubscribe();
  }, []);

  const onControlBtnClick = (message) => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: message }),
    );
  };

  const onFormSubmitHandler = (data) => {
    setOpenModal(false);
    if (data) {
      // Intentionally left blank or for future modals
    }
    modalKey.current = null;
    filesModalType.current = null;
    isFilesModalWithInput.current = null;
    filesModalHeader.current = null;
    filesModalPlaceholder.current = null;
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
            <h1 className="text-xl font-bold text-white tracking-wide">Robot Control</h1>
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
        </div>

        {/* MIDDLE: Primary Map Area */}
        <div className="flex-1 min-h-0 relative bg-slate-950 p-2">
          <div className="w-full h-full rounded-md overflow-hidden bg-white shadow-inner flex items-center justify-center">
            <Map />
          </div>
        </div>

        {/* BOTTOM: Split Panel (Messages | Controls | Step/Start/Stop) */}
        <div className="h-[40vh] min-h-[300px] flex-shrink-0 bg-slate-900 flex flex-row w-full border-t border-slate-800">

          {/* 1. Messages Panel (Left) */}
          <div className="flex-1 flex flex-col min-w-0 border-r border-slate-800 p-4">
            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-2">Messages</span>
            <div className="flex-1 bg-slate-950 rounded-md p-3 overflow-auto border border-slate-800 text-sm">
              <Logs />
            </div>
          </div>

          {/* 2. Controls Panel (Center) */}
          <div className="w-[300px] flex-shrink-0 flex flex-col p-4 border-r border-slate-800 overflow-y-auto">
            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider block mb-3">Navigation</span>
            <div className="grid grid-cols-3 gap-2 mb-6">
              <Button size="small" onBtnClick={() => onControlBtnClick("next_point")}>
                Next
              </Button>
              <Button size="small" onBtnClick={() => onControlBtnClick("previous_point")}>
                Prev
              </Button>
              <Button size="small" onBtnClick={() => onControlBtnClick("home")}>
                Collect Data + Rod extend/retract + loop
              </Button>
            </div>

            <div className="h-px bg-slate-800 w-full mb-6"></div>

            <span className="text-xs font-bold text-slate-400 uppercase tracking-wider block mb-3">Rod Control</span>
            <div className="grid grid-cols-2 gap-2">
              <Button size="small" onBtnClick={() => onControlBtnClick("rod_extend")}>
                Extend
              </Button>
              <Button size="small" onBtnClick={() => onControlBtnClick("rod_retract")}>
                Retract
              </Button>
            </div>
          </div>

          {/* 3. Start / Step / Stop Panel (Right) — 3 round buttons */}
          <div className="w-[120px] flex-shrink-0 flex flex-col items-center justify-center gap-6 p-2 border-l border-slate-800 bg-slate-900/50">

            {/* START BUTTON */}
            <div className="flex flex-col items-center gap-2">
              {/* 1. The bounding box: The layout ONLY sees a 64x64px square */}
              <div className="relative w-16 h-16 flex items-center justify-center">
                {/* 2. Absolute positioning removes the giant original button from the flex layout */}
                <div className="absolute transform scale-[0.55] hover:scale-[0.6] transition-transform origin-center flex items-center justify-center">
                  <RoundedButton
                    type="success"
                    onBtnClick={() => onControlBtnClick("start")}
                  >
                    Start
                  </RoundedButton>
                </div>
              </div>
              <span className="text-[10px] text-slate-400 font-bold uppercase tracking-wider text-center">Full Route</span>
            </div>

            {/* STEP BUTTON */}
            <div className="flex flex-col items-center gap-2">
              <div className="relative w-16 h-16 flex items-center justify-center">
                <div className="absolute transform scale-[0.55] hover:scale-[0.6] transition-transform origin-center flex items-center justify-center">
                  <RoundedButton
                    type="orange"
                    onBtnClick={() => onControlBtnClick("next_point")}
                  >
                    Step
                  </RoundedButton>
                </div>
              </div>
              <span className="text-[10px] text-slate-400 font-bold uppercase tracking-wider text-center">Next WP</span>
            </div>

            {/* STOP BUTTON */}
            <div className="flex flex-col items-center gap-2">
              <div className="relative w-16 h-16 flex items-center justify-center">
                <div className="absolute transform scale-[0.55] hover:scale-[0.6] transition-transform origin-center flex items-center justify-center">
                  <RoundedButton
                    type="danger"
                    onBtnClick={() => onControlBtnClick("stop")}
                  >
                    Stop
                  </RoundedButton>
                </div>
              </div>
              <span className="text-[10px] text-slate-400 font-bold uppercase tracking-wider text-center">Emergency</span>
            </div>

          </div>
        </div>

      </div>
    </>
  );
};

export default ControlPage;