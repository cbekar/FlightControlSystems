%% Trim
global S Tmil engineCount engineLUT;
Zinit('737');
optimalTrimIndex = trim('737PDT','MMCR');
selectedTrimIndex = 20;
fillSvalues(selectedTrimIndex);
sim('B733_JSB');
%% Linearization
linearization('B733_JSB');
%% Controller designs
% 1) Find SAS gains
% 1-A) Longitudinal
% Pitch Damper
getGainLUTs();
%%
close all