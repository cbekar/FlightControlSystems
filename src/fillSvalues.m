function fillSvalues(trimInd)
global S
S.simtime = 300;
timeVector = (0:S.simtime * 1/0.01 +1) * 0.01;
S.usignal(:,1) = timeVector';
S.usignal(:,2:5) = S.trim(trimInd,3:6).*ones(1,size(timeVector,2))';
S.xu = [S.trim(trimInd,8),S.trim(trimInd,1),S.trim(trimInd,2),...
    S.trim(trimInd,7),S.trim(trimInd,1)+S.states.gamma,0,0,0,0,0,0,S.trim(trimInd,9)]; 
S.uu = [S.trim(trimInd,3),S.trim(trimInd,4),S.trim(trimInd,5),S.trim(trimInd,6)]; 
end