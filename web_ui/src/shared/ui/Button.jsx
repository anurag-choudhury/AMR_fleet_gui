import React from "react";

const Button = ({ children, onBtnClick, type }) => {
  let buttonClasses = "";
  switch (type) {
    case "gray":
      buttonClasses =
        "bg-slate-700 hover:bg-slate-600 active:bg-slate-500 text-white";
      break;
    case "orange":
      buttonClasses =
        "bg-blue-700 hover:bg-blue-600 active:bg-blue-500 text-white";
      break;
    case "disabled":
      buttonClasses =
        "border border-slate-700 opacity-40 cursor-not-allowed bg-slate-800 text-slate-500";
      break;
    default:
      buttonClasses =
        "border border-slate-600 hover:bg-slate-700 active:bg-slate-600 bg-slate-800 text-slate-200";
  }

  return (
    <button
      className={`px-3 py-2 ${buttonClasses} flex w-full items-center justify-center gap-2 rounded-lg text-sm font-medium transition-colors duration-150`}
      onClick={type !== "disabled" ? onBtnClick : () => {}}
    >
      {children}
    </button>
  );
};
export default Button;
