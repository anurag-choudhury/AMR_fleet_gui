import React from "react";
import { NavLink } from "react-router-dom";

const Header = () => {
  return (
    <header className="flex items-center justify-center bg-slate-900 border-b border-slate-800 px-4 sm:px-6 flex-shrink-0">
      <nav className="flex gap-8 sm:gap-12 lg:gap-16 py-3 text-sm sm:text-base lg:text-lg font-medium">
        <NavLink
          to="/"
          className={({ isActive }) =>
            isActive
              ? "text-blue-400 border-b-2 border-blue-400 pb-1 transition-colors"
              : "text-slate-300 hover:text-white transition-colors pb-1"
          }
        >
          Map
        </NavLink>
        <NavLink
          to="/route"
          className={({ isActive }) =>
            isActive
              ? "text-blue-400 border-b-2 border-blue-400 pb-1 transition-colors"
              : "text-slate-300 hover:text-white transition-colors pb-1"
          }
        >
          Route
        </NavLink>
        <NavLink
          to="/control"
          className={({ isActive }) =>
            isActive
              ? "text-blue-400 border-b-2 border-blue-400 pb-1 transition-colors"
              : "text-slate-300 hover:text-white transition-colors pb-1"
          }
        >
          Control
        </NavLink>
        <NavLink
          to="/info"
          className={({ isActive }) =>
            isActive
              ? "text-blue-400 border-b-2 border-blue-400 pb-1 transition-colors"
              : "text-slate-300 hover:text-white transition-colors pb-1"
          }
        >
          Info
        </NavLink>
      </nav>
    </header>
  );
};

export default Header;
