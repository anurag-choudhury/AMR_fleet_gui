import React, { useContext, useRef } from "react";

import { AppConfig } from "../shared/constants/index";
import { RosContext } from "../app/App";

import Map from "../components/Map";
import Logs from "../components/Logs";

import Button from "../shared/ui/Button";
import RoundedButton from "../shared/ui/RoundedButton";

const ControlPage = () => {
  const ros = useContext(RosContext);

  const uiOperationTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const onControlBtnClick = (message) => {
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: message }),
    );
  };

  return (
    <div className="sectionHeight flex flex-col items-stretch gap-8 px-4 pb-6 pt-6 sm:px-6 md:gap-10 md:pt-[30px] xl:flex-row xl:px-8">
      {/* LEFT: CONTROL */}
      <section className="color-white mb-auto flex w-full flex-col justify-center xl:w-[65%]">
        <h3 className="w-full text-center font-[RobotoMono] text-2xl font-bold text-white sm:text-3xl">
          Control
        </h3>

        <div className="mt-6 flex w-full flex-1 flex-col items-stretch justify-evenly gap-6 xl:mt-[30px] xl:flex-row xl:items-start xl:gap-10 xl:justify-between">
          {/* Main buttons */}
          <div className="flex flex-1 flex-col gap-4 2xl:gap-24">
            {/* Keep PC structure: still a centered column; on mobile make it full width */}
            <div className="grid w-full grid-cols-1 gap-4 self-center sm:w-3/4 md:w-2/3 xl:w-1/2 xl:gap-10">
              {/* <Button onBtnClick={() => onControlBtnClick("follow_route")}>
                <span className="mx-auto">Follow</span>
              </Button> */}
              <Button onBtnClick={() => onControlBtnClick("next_point")}>
                <span className="mx-auto">Next point</span>
              </Button>
              <Button onBtnClick={() => onControlBtnClick("previous_point")}>
                <span className="mx-auto">Prev point</span>
              </Button>
              <Button size="big" onBtnClick={() => onControlBtnClick("home")}>
                <span className="mx-auto">Home</span>
              </Button>
              <Button onBtnClick={() => onControlBtnClick("rod_extend")}>
                <span className="mx-auto">Extend Rod</span>
              </Button>

              <Button onBtnClick={() => onControlBtnClick("rod_retract")}>
                <span className="mx-auto">Retract Rod</span>
              </Button>

            </div>
          </div>

          {/* Start/Stop for XL+ (same as your current PC structure) */}
          <div className="hidden flex-col justify-between gap-[100px] xl:flex">
            <RoundedButton
              type="success"
              onBtnClick={() => onControlBtnClick("start")}
            >
              Start
            </RoundedButton>
            <RoundedButton
              type="danger"
              onBtnClick={() => onControlBtnClick("stop")}
            >
              Stop
            </RoundedButton>
          </div>
        </div>
      </section>

      {/* RIGHT: MAP */}
      <section className="color-white mb-auto mt-2 flex w-full flex-col items-center justify-center gap-6 xl:mt-0 xl:w-[40%] xl:gap-7">
        <h3 className="w-full text-center font-[RobotoMono] text-2xl font-bold text-white sm:text-3xl">
          Map
        </h3>

        {/* Responsive map height (keeps 400px on md+ like your PC view) */}
        <div className="h-[260px] w-full sm:h-[320px] md:h-[400px]">
          <Map />
        </div>

        {/* Logs: scrollable on small, fixed-ish on desktop */}
        <div className="w-full">
          <div className="max-h-[200px] min-h-[140px] overflow-auto md:h-[165px] md:max-h-none md:min-h-0">
            <Logs />
          </div>
        </div>

        {/* Start/Stop for mobile/tablet (unchanged behavior) */}
        <div className="flex w-full justify-center gap-6 sm:justify-between sm:gap-10 xl:hidden">
          <RoundedButton
            type="success"
            onBtnClick={() => onControlBtnClick("start")}
          >
            Start
          </RoundedButton>
          <RoundedButton
            type="danger"
            onBtnClick={() => onControlBtnClick("stop")}
          >
            Stop
          </RoundedButton>
        </div>
      </section>
    </div>
  );
};

export default ControlPage;
