%% Configuration of main function
% Fast mode: The global variable S is loaded from file (default: fast mode)
% Slow mode: S is generated from scratch
clc; clear; fast = 0; simulink = 0; 
global S Tmil engineLUT engineCount;
%% Initialization of nonlinear model for selected aircraft from JSB_SIM
% Tested only for B737!
Zinit('737');
if fast == 0
    %% Trimming the model for selected flight conditions with 26 FLs
    % (default: med. mass, cruise)
    flightcondition  = 'MMCR';
    optimalTrimIndex = trim('737PDT',flightcondition);
    %% Run simulink for selected trim conditions
    selectedTrimIndex = 20;
    fillSvalues(selectedTrimIndex);
    sim('B733_JSB');
    %% Linearization
    linearization('B733_JSB');
    %% Controller designs
    modern = 0;
    getGainLUTs(modern);
    save(['mat/S_733_',flightcondition])
else
    load('S_733_MMCR.mat');
end
if simulink == 1
    fillSvalues(selectedTrimIndex);
    sim('B733_JSB');
    sim('SAS');
    sim('CAS');
    sim('AP_PAH');
end