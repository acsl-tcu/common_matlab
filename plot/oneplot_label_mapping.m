function labelmap = oneplot_label_mapping
%oneplot_label_mapping プロット関連のラベル管理をする関数
%   詳細説明をここに記述

labelmap.p.x = {"Time [s]"};
labelmap.p.y = {"Position [m]"};


labelmap.v.x = {"Time [s]"};
labelmap.v.y = {"Velocity [m/s]"};


labelmap.q.x = {"Time [s]"};
labelmap.q.y = {"Angle [rad]"};


labelmap.w.x = {"Time [s]"};
labelmap.w.y = {"Angular velocity [rad/s]"};


labelmap.input.x = {"Time [s]"};
labelmap.input.y = {"Controller input [N, Nm]"};


labelmap.input24.x = {"Time [s]"};
labelmap.input24.y = {"Controller input [Nm]"};


labelmap.inner_input.x = {"Time [s]"};
labelmap.inner_input.y = {"Transmitter input [N, Nm]"};


labelmap.inner_input24.x = {"Time [s]"};
labelmap.inner_input24.y = {"Transmitter input [Nm]"};



labelmap.p1_p2.x = {"$x$ [m]"};
labelmap.p1_p2.y = {"$y$ [m]"};

labelmap.p1_p2_p3.x ={"$x$ [m]"};
labelmap.p1_p2_p3.y ={"$y$ [m]"};
labelmap.p1_p2_p3.z ={"$z$ [m]"};

end
