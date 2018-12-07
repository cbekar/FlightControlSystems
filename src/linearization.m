function linearization(model)
global S
for i=1:26
    fillSvalues(i);
    S.linear(i) = linmod(model,S.xu,S.uu); 
    progressbar(i/26);
end
close all;