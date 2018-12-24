function K = SAS_Pitch_EigAsgn(A,B,silent)         
    % x_states = [alpha;q;theta;de]         
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
    %         C = [0 1 0 0;
    %              47.76 -0.268 0 -4.56;
    %              zeros(2,2) eye(2)
    %              ];
    C = eye(4);
    D = zeros(4,1);      
    %% 
    % Check Controllability   
    Co = [B A*B A^2*B A^3*B];
    unco = length(A) - rank(Co);
    if silent == 0
        if (unco == 0)
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
    deigs = [ev1 ev2 -1 -9]'; %Desired Eigenvalues
    % Gain Calculation
    K = place(A,B,deigs);
    %%
    %Verifying Design
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