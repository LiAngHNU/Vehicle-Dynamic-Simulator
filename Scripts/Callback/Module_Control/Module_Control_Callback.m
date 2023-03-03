%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Callback Function of Module Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/12/07 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Module Path %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PATH_Control = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control";
PATH_PreProcessor = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control/PreProcessor";
PATH_Controller = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control/Controller";
PATH_PostProcessor = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control/PostProcessor";
PATH_Lat_Controller = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control/Controller/Lat_Controller";
PATH_Lon_Controller = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control/Controller/Lon_Controller";
PATH_Ver_Controller = ...
    "Vehicle_Dynamic_Simulator/v0.40c/MODULE::AutoPilot/MODULE::Control/Controller/Ver_Controller";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Get FLAGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FLAG_VEHICLE_ = get_param("Vehicle_Dynamic_Simulator/v0.40c","FLAG_VEHICLE_");
FLAG_TRACKING_TARGET_ = get_param(PATH_Control, 'FLAG_TRACKING_TARGET_');
FLAG_LON_CONTROLLER_ = get_param(PATH_Control,'FLAG_LON_CONTROLLER_');
FLAG_LAT_CONTROLLER_ = get_param(PATH_Control,'FLAG_LAT_CONTROLLER_');
FLAG_VER_CONTROLLER_ = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Tracking Mode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch FLAG_TRACKING_TARGET_
    case 'Reference'
        set_param([PATH_PreProcessor+'/getFrenetErrorsOnReference'],'Commented','off');
        set_param([PATH_PreProcessor+'/getFrenetErrorsOnTrajectory'],'Commented','on');
    case 'Trajectory'
        set_param([PATH_PreProcessor+'/getFrenetErrorsOnReference'],'Commented','on');
        set_param([PATH_PreProcessor+'/getFrenetErrorsOnTrajectory'],'Commented','off');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Lateral Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch FLAG_LAT_CONTROLLER_
    case "Manual"
        set_param([PATH_Lat_Controller+'/Manual_Controller'],'Commented','off');
        set_param([PATH_Lat_Controller+'/LQR_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MRAC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPPI_Based_Controller'],'Commented','on');
    case "LQR Based Algorithms"
        set_param([PATH_Lat_Controller+'/Manual_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/LQR_Based_Controller'],'Commented','off');
        set_param([PATH_Lat_Controller+'/MPC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MRAC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPPI_Based_Controller'],'Commented','on');
        FLAG_LQR_TYPE_ = get_param([PATH_Lat_Controller+'/LQR_Based_Controller'],'FLAG_LQR_TYPE_');
        switch FLAG_LQR_TYPE_
            case 'Online Standard LQR Controller'
            case 'Online Compensated LQR Controller'
            case 'Online Compensated OPC Controller'
            case 'Offline Compensated OPC Controller'
                load("Configs\Audi_R8_Etron\OpcGainInfo.mat");
        end
    case "MPC Based Algorithms"
        set_param([PATH_Lat_Controller+'/Manual_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/LQR_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPC_Based_Controller'],'Commented','off');
        set_param([PATH_Lat_Controller+'/MRAC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPPI_Based_Controller'],'Commented','on');
    case "MRAC Based Algorithms"
        set_param([PATH_Lat_Controller+'/Manual_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/LQR_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MRAC_Based_Controller'],'Commented','off');
        set_param([PATH_Lat_Controller+'/MPPI_Based_Controller'],'Commented','on');
    case "MPPI Based Algorithms"
        set_param([PATH_Lat_Controller+'/Manual_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/LQR_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MRAC_Based_Controller'],'Commented','on');
        set_param([PATH_Lat_Controller+'/MPPI_Based_Controller'],'Commented','off');
end