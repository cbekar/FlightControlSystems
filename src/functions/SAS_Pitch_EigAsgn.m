function K = SAS_Pitch_EigAsgn(A,B,silent)                
    A = [ A(2,2) A(2,5) A(2,8) -B(2,2);
          A(5,2) A(5,5) A(5,8) -B(5,2);
          A(8,2) A(8,5) A(8,8) -B(8,2);
          0 0 0 -10;
        ];
    B = [ 0;
          0;
          0;
          10;
        ];
    C = eye(4); D = zeros(4,1);      
    %% Check Controllability
    unco = length(A) - rank(ctrb(A,B));
    if unco == 0
        if silent == 0
            sprintf('System is Controllable')
        end
    else
        error('System is Not Controllable')
    end
    %% Desired EigenValues Calculation
    damp = 0.8; 
    wn = 3; 
    ev1 = -damp*wn + wn*sqrt(damp^2-1); 
    ev2 = -damp*wn - wn*sqrt(damp^2-1); 
    deigs = [ev1 ev2 -1 -9]'; 
    %% Gain Calculation
    K = place(A,B,deigs);
    %% Verifying Design
    if silent == 0
        Ac = A-B*K;
        sys1 = ss(Ac,B,C,D);
        eig(sys1)
        %Analyze Design       
        subplot(231);
        rlocus(sys1(1,1))
        grid on;
        subplot(232);
        rlocus(sys1(2,1))
        grid on;
        subplot(233);
        rlocus(sys1(3,1))
        grid on;
        %Impulse response
        subplot(2,3,[4 5 6]);
        impulse(sys1(1:3,:));
        grid on
    end
end