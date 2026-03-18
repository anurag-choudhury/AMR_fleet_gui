import React from "react";

import { Outlet } from "react-router-dom";
import Header from "../components/Header";

const AppLayout = () => {
  return (
    <div className="flex flex-col h-screen overflow-hidden">
      <Header />
      <main className="flex flex-col flex-1 min-h-0 overflow-hidden">
        <Outlet />
      </main>
    </div>
  );
};
export default AppLayout;
