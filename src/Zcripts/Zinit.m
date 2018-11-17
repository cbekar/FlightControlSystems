function Zinit(model)
clc; clear all
global Tmil engineCount engineLUT
addpath('src');
addpath('data');
addpath('img');
%TODO check for other xmls 
%This is the function that the model preloads, adding the necessary structs
%and vars from parsed xmls to the workspace for the consumption by the 
%model.
aircraft_params = xml2struct('737.xml');
engineCount = size(aircraft_params(1).fdm_config.propulsion.engine,2);
engine_params = xml2struct(strcat(aircraft_params(1).fdm_config.propulsion.engine{1}.Attributes.file, '.xml'));
fn = fieldnames(engine_params);
Tmil = str2double(engine_params.(fn{1}).milthrust.Text);
engineLUT = reshape([[0.0420  0.0436  0.0528  0.0694  0.0899  0.1183  0.1467;0.0500  0.0501  0.0335  0.0544  0.0797  0.1049  0.1342;0.0040  0.0047  0.0020  0.0272  0.0595  0.0891  0.1203;0.0     0.0     0.0     0.0     0.0276  0.0718  0.1073;0.0     0.0     0.0     0.0     0.0174  0.0468  0.0900;0.0     0.0     0.0     0.0     0.0     0.0422  0.0700] [1.2600  1.0000  0.7400  0.5340  0.3720  0.2410  0.1490;1.1710  0.9340  0.6970  0.5060  0.3550  0.2310  0.1430;1.1500  0.9210  0.6920  0.5060  0.3570  0.2330  0.1450;1.1810  0.9510  0.7210  0.5320  0.3780  0.2480  0.1540;1.2580  1.0200  0.7820  0.5820  0.4170  0.2750  0.1700;1.3690  1.1200  0.8710  0.6510  0.4750  0.3150  0.1950]],[6,7,2]);
end