function K = SAS_RollYaw_EigAsgn(A,B,silent)
    % x_states = [beta;phi;p;r;da;dr]
    A = [ A(3,3) A(3,4) A(3,7) A(3,9) -B(3,3) -B(3,4);
          A(4,3) A(4,4) A(4,7) A(4,9) -B(4,3) -B(4,4);
          A(7,3) A(7,4) A(7,7) A(7,9) -B(7,3) -B(7,4);
          A(9,3) A(9,4) A(9,7) A(9,9) -B(9,3) -B(9,4);
          0 0 0 0 -10 0;
          0 0 0 0 0 -10
        ];
    B = [ 0 0;
          0 0;
          0 0;
          0 0;
          10 0;
          0 10
        ];
    C = eye(6);
    D = zeros(6,2);
%     tauWash = 1;
%     aw   = -1/tauWash; bw= [0 0 0 0 0 1/tauWash];
%     cw   = [0; 0; 0; 0; 0;-1]; dw= eye(6);           % y1=p y2=washed-r
%     wash = ss(aw,bw,cw,dw);
%     [A,B,C,D] = ssdata(series(ss(A,B,C,D),wash)); % x1=wash, x2=beta,.., x6=ail, x7=rdr
    %%
    % Check Controllability   
    Co = [B A*B A^2*B A^3*B A^4*B A^5*B];
    unco = length(A) - rank(Co);
    if silent == 0
        if unco == 0
        sprintf('System is Controllable')
        else 
            sprintf('System is Not Controllable')
        end
    end
    % Desired EigenValues Calculation
    damp = 0.8; %Damping Ratio
    wn = 3; %Natural Frequency  
    ev1 = -damp*wn + wn*sqrt(damp^2-1); %Desired Eigenvalue1
    ev2 = -damp*wn - wn*sqrt(damp^2-1); %Desired Eigenvalue2
    wn = 2;
    ev3 = -damp*wn + wn*sqrt(damp^2-1); %Desired Eigenvalue1
    ev4 = -damp*wn - wn*sqrt(damp^2-1); %Desired Eigenvalue2
    deigs = [ev1 ev2 ev3 ev4 -10 -10]'; %Desired Eigenvalues
    % Gain Calculation
    K = place(A,B,deigs);
    %%
    %Verifying Design
    if silent == 0
        Ac = A-B*K;
        sys1 = ss(Ac,B,C,D);
        eig(sys1)
        %Analyze Design       
        subplot(421);
        rlocus(sys1(1,1))
        subplot(422);
        rlocus(sys1(1,2))
        subplot(423);
        rlocus(sys1(2,1))
        subplot(424);
        rlocus(sys1(2,2))
        subplot(425);
        rlocus(sys1(3,1))
        subplot(426);
        rlocus(sys1(3,2))
        %Impulse response
        subplot(4,2,[7 8]);
        impulse(sys1(1:3,:))
        grid on
    end
end