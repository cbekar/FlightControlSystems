clc; clear; 
global S Tmil engineLUT engineCount;
%% Configuration of main function
% Fast mode: The global variable S is loaded from file (default: fast mode)
% Slow mode: S is generated from scratch
% modern:    0 = Classical Control (incomplete), 1 = EigStructure, 2 = LQR
fast = 1; simulink = 1; modern = 1; selectedTrimIndex = 20;
%% Initialization of nonlinear model for selected aircraft from JSB_SIM
% Tested only for B737!
Zinit('737');
if fast == 0
    %% Trimming the model for selected flight conditions with 26 FLs
    % (default: med. mass, cruise)
    flightcondition  = 'MMCR';
    optimalTrimIndex = trim(strcat(S.model,'PDT'),flightcondition);
    %% Run simulink for selected trim conditions
    fillSvalues(selectedTrimIndex);
    sim('B733_JSB');
    %% Linearization
    linearization('B733_JSB');
    %% Controller designs
    getGainLUTs(modern);
    save(['mat/B733_',flightcondition])
else
    load('B733_MMCR.mat');
end
fillSvalues(selectedTrimIndex);
sim('B733_JSB');
fillSvalues(selectedTrimIndex); % necessary?
sim('B733_SAS');
sim('B733_CAS');
sim('B733_AP_PAH');