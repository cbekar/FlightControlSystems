function K = SAS_RollYaw_LQR(A,B,silent)
    Kold = SAS_RollYaw_EigAsgn(A,B,silent);
    % x_states = [beta;phi;p;r;dr;da]
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
    sys0 = ss(A-B*Kold*C,B,C,D);
    if silent == 0
        damp(sys0)
    end
    Q = diag([35 5 5 35 0 0]);
    R = 10*eye(2);
    for k = 0:1000
        Ac = A - B*Kold*C;
        P = lyap(Ac',C'*Kold'*R*Kold*C + Q);
        % Ac'*P + P*Ac + C'*K'*R*K*C + Q  % verify of P
        X = eye (6);
        S = lyap(Ac,X);
        %Ac*S + S*Ac' + X   % verify of S
        Jnew = (1/2)*trace(P*X);
        deltaK = R\B'*P*S*C'/(C*S*C')-Kold;
        Knew = Kold + 0.2*deltaK;
        if k == 0
           k = k+1;
           Jold = Jnew;
           Kold = Knew;
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
    if silent == 0
        sys1 = ss(Ac,B,C,D);
        eigs = eig(sys1);
        damp(sys1);
    end
 end 