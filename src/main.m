%% Configuration of main function
% (default: fast mode)
% Fast mode: The global variable S is loaded from file
% Slow mode: S is generated from scratch
clc; clear all
fast = 1;
%% Initialization of nonlinear model for selected aircraft from JSB_SIM
% Tested only for B737!
global S Tmil engineCount engineLUT;
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
    getGainLUTs();
    save(['mat/S_733_',flightcondition])
else
    load('S_733_MMCR.mat');
    fillSvalues(selectedTrimIndex);
end