%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Callback Function of Module Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Version: v1.0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date: 2022/12/01 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Li.Ang %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Email: li.ang@cidi.ai %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SimulinkModel = "Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation";
FLAG_MODE_ = get_param(SimulinkModel,'FLAG_MODE_');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Simulation Mode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch FLAG_MODE_
    case "CarSim"
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function','Commented','off');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function_video','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function_video','Commented','on');
    case "CarSim Real-Time"
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function_video','Commented','off');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function_video','Commented','on');
    case "TruckSim"
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function_video','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function','Commented','off');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function_video','Commented','on');
    case "TruckSim Real-Time"
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/CarSim S-Function_video','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function','Commented','on');
        set_param('Vehicle_Dynamic_Simulator/v0.40b/MODULE::Simulation/TruckSim S-Function_video','Commented','off');
end