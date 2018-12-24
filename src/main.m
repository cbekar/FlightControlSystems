%% author: u. can bekar, aero phd stu
clc; clear; global S;
%% Configuration of main function
% Fast mode: The global variable S is loaded from file (default: fast mode)
% modern: 0 -> Classical Control, 1 -> EigStructure, 2 -> LQR
airframe = 'B737'; fast = 0; modern = 1; oppoint = 20; FC  = 'MMCR';
%% Initialization of nonlinear model for selected aircraft from JSB_SIM
Zinit(airframe);
%% Fast/Slow switch
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
end
%% Sims
fillSvalues(oppoint);
sim(strcat(airframe,'_JSB'));
sim(strcat(airframe,'_SAS'));
% sim(strcat(airframe,'_CAS'));
% sim(strcat(airframe,'_PAH'));
close all;