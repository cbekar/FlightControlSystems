function K = SAS_Pitch_LQR(A,B,silent)
    Kold = SAS_Pitch_EigAsgn(A,B,silent);
    % x_states = [alpha;q;theta;de]         
    A = [ A(2,2) A(2,8) A(2,5) -B(2,2);
          A(8,2) A(8,8) A(8,5) -B(8,2);
          A(5,2) A(5,8) A(5,5) -B(5,2);
          0 0 0 -10      
        ];
    B = [ 0;
          0;
          0; 
          10 
        ];
    C = eye(4);
    D = zeros(4,1);

    sys0 = ss(A-B*Kold*C,B,C,D);
    damp(sys0)
    Q = diag([35 5 5 0]);
    R = 10*eye(1);
    for k = 0:1000
        Ac = A - B*Kold*C;
        P = lyap(Ac',C'*Kold'*R*Kold*C + Q);
        % Ac'*P + P*Ac + C'*K'*R*K*C + Q  % verify of P
        X = eye (4);
        S = lyap(Ac,X);
        %Ac*S + S*Ac' + X   % verify of S
        Jnew = (1/2)*trace(P*X);
        deltaK = R\B'*P*S*C'/(C*S*C')-Kold;
        Knew = Kold + 0.2*deltaK;
        if k == 0
           k = k+1;
           Jold=Jnew;
           Kold=Knew;
        else  
            if abs(Jnew-Jold) >= 0.001
                Jold=Jnew;
                Kold = Knew;
                k = k+1;
            else
                K = Knew;
                J = Jnew;
                k = 1000;
            end 
        end
    end 
    sys1 = ss(Ac,B,C,D);
    eigs = eig(sys1);
    damp(sys1)
end