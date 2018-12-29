function [K] = LQR(A,B,C,D,Q,R,K0)
    Kold = K0; Jold = -Inf; alpha = 0.2;
    for k = 0:1000
        Ac = A - B*Kold*C;
        P = lyap(Ac',C'*Kold'*R*Kold*C + Q);
        X = eye(size(C,2));
        S = lyap(Ac,X);
        Jnew = (1/2)*trace(P*X);
        deltaK = R\B'*P*S*C'/(C*S*C')-Kold;
        % choose alpha s.t. Acl_new is stable
        Knew = Kold + alpha*deltaK;
        stab = isstable(ss(A-B*Knew*C,B,C,D));
        if stab == 1
            if (abs(Jnew-Jold) >= eps)
                Jold = Jnew;
                Kold = Knew;
            else
                K = Knew;
                break;
            end 
        else
            alpha = alpha - 0.05;
        end
    end
end