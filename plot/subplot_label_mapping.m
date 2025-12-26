function labelmap = subplot_label_mapping
%subplot_label_mapping プロット関連のラベル管理をする関数
%   詳細説明をここに記述

labelmap.p.x = {"Time [s]"};
labelmap.p.y = {...
    "$x$ [m]";
    "$y$ [m]";
    "$z$ [m]";
    };


labelmap.v.x = {"Time [s]"};
labelmap.v.y = {...
    "$v_x$ [m/s]";
    "$v_y$ [m/s]";
    "$v_z$ [m/s]";
    };


labelmap.q.x = {"Time [s]"};
labelmap.q.y = {...
    "$\phi$ [rad]";
    "$\theta$ [rad]";
    "$\psi$ [rad]";
    };


labelmap.w.x = {"Time [s]"};
labelmap.w.y = {...
    "$\Omega_{roll}$ [rad/s]";
    "$\Omega_{pitch}$ [rad/s]";
    "$\Omega_{yaw}$ [rad/s]";
    };


labelmap.input.x = {"Time [s]"};
labelmap.input.y = {...
    "$T$ [N]";
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };


labelmap.input24.x = {"Time [s]"};
labelmap.input24.y = {...
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };


labelmap.inner_input.x = {"Time [s]"};
labelmap.inner_input.y = {...
    "$T$ [N]";
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };


labelmap.inner_input24.x = {"Time [s]"};
labelmap.inner_input24.y = {...
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };



labelmap.p1_p2.x = {"$x$ [m]"};
labelmap.p1_p2.y = {"$y$ [m]"};

labelmap.p1_p2_p3.x ={"$x$ [m]"};
labelmap.p1_p2_p3.y ={"$y$ [m]"};
labelmap.p1_p2_p3.z ={"$z$ [m]"};

end
