%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Callback Function of Module Localization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/12/12 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SimulinkModel = "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Localization";
FLAG_NOISE_GPS_ = get_param(SimulinkModel,'FLAG_NOISE_GPS_');
FLAG_NOISE_IMU_ = get_param(SimulinkModel,'FLAG_NOISE_IMU_');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Noise Mode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch FLAG_NOISE_GPS_
    case 'on'
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_LP'],'Commented','off');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_AP'],'Commented','off');
    case 'off'
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_LP'],'Commented','on');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_AP'],'Commented','on');
end
switch FLAG_NOISE_IMU_
    case 'on'
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_LV'],'Commented','off');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_AV'],'Commented','off');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_LA'],'Commented','off');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_AA'],'Commented','off');
    case 'off'
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_LV'],'Commented','on');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_AV'],'Commented','on');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_LA'],'Commented','on');
        set_param(['Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/' ...
            'MODULE::Localization/Navigation/Noise_AA'],'Commented','on');
end