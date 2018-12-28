%% author: u. can bekar, aero phd stu
clc; clear; global S;
%% Configuration of main function
% Fast mode: The global variable S is loaded from file (default: fast mode)
% modern: 0 -> Classical Control, 1 -> EigStructure, 2 -> LQR
airframe = 'B737'; fast = 1; modern = 2; oppoint = 20; FC  = 'MMCR';
%% Initialization of nonlinear model for selected aircraft from JSB_SIM
Zinit(airframe);
if fast == 0
    %% Trimming the model for selected flight conditions with 26 FLs
    trim(strcat(airframe,'PDT'),FC); 
    %% Linearization around various oppoints
    linearization(strcat(airframe,'_JSB_0')); %_0 model has no animation
    %% Controllers' gain 
    getGainLUTs(modern);
    save(strcat('artifacts/mat/',airframe,'_',FC,'_',string(modern)));
else
    load(strcat(airframe,'_',FC,'_',string(modern),'.mat'));
    getGainLUTs(modern);
%     save(strcat('artifacts/mat/',airframe,'_',FC,'_',string(modern)));
end
%% Sims
fillSvalues(oppoint);
sim(strcat(airframe,'_JSB'));
sim(strcat(airframe,'_SAS'));
% sim(strcat(airframe,'_CAS'));
% sim(strcat(airframe,'_PAH'));
close all;
%% Future Work
% One-click autopilots design for various JSB airframes (other than B737)
% FlightGear integration