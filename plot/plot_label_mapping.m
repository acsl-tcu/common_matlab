function labelmap = plot_label_mapping
%plot_label_mapping プロット関連のラベル管理をする関数
%   詳細説明をここに記述

labelmap.p = {...
    "$x$ [m]";
    "$y$ [m]";
    "$z$ [m]";
    };

labelmap.v = {...
    "$v_x$ [m/s]";
    "$v_y$ [m/s]";
    "$v_z$ [m/s]";
    };

labelmap.q = {...
    "$\phi$ [rad]";
    "$\theta$ [rad]";
    "$\psi$ [rad]";
    };

labelmap.w = {...
    "$\Omega_{roll}$ [rad/s]";
    "$\Omega_{pitch}$ [rad/s]";
    "$\Omega_{yaw}$ [rad/s]";
    };

labelmap.input = {...
    "$T$ [N]";
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };

labelmap.input24 = {...
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };

labelmap.inner_input = {...
    "$T$ [N]";
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };

labelmap.inner_input24 = {...
    "$\tau_{roll}$ [Nm]";
    "$\tau_{pitch}$ [Nm]";
    "$\tau_{yaw}$ [Nm]";
    };

labelmap.p1_p2 = {...
    "estimator";
    "reference";
    };

labelmap.p1_p2_p3 = {...
    "estimator";
    "reference";
    };

end
