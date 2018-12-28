function [K] = LQR(A,B,C,D,Q,R,K0)
%LQR Summary of this function goes here
%   Detailed explanation goes here
    flag = 0; Kold = K0; Jold = -Inf;
    for k = 0:1000
        if flag == 0
            Ac = A - B*Kold*C;
            P = lyap(Ac',C'*Kold'*R*Kold*C + Q);
            X = eye (size(C,2));
            S = lyap(Ac,X);
            Jnew = (1/2)*trace(P*X);
            deltaK = R\B'*P*S*C'/(C*S*C')-Kold;
            % choose alpha s.t. Acl_new is stable
            for alpha = 0.5:-0.05:0.01
                Knew = Kold + alpha*deltaK;
                stab = isstable(ss(A-B*Knew*C,B,C,D));
                if stab == 1 
                    if (abs(Jnew-Jold) >= eps)
                    Jold = Jnew;
                    Kold = Knew;
                    else
                        K = Knew;
                        flag = 1;
                        break;
                    end 
                end
            end
        else
            break;
        end
    end 
end