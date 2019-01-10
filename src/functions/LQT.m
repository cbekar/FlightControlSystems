function [K] = LQT(A,B,C,D,F,G,Q,R,K0)
    Kold = K0; Jold = -Inf; alpha = 0.2; K = K0;
    for k = 0:1000
        Ac = A - B*Kold*C;
        Bc = G - B*Kold*F;
        P = lyap(Ac',C'*Kold'*R*Kold*C + Q);
        r0 = 1;
        xb = -Ac\Bc*r0;  
        Aci = inv(Ac);
        X = Aci*Bc*r0*r0'*Bc'*Aci';
        S = lyap(Ac,X);
        H = chol(Q);
        eb = (1+H*Aci*Bc)*r0;
        yb = C*xb+F*r0;
        %V = eye(size(eb,1));%V=0 when feedforward has only integrators
        Jnew = (1/2)*trace(P*X);%+(1/2)*(eb'*V*eb);
        %deltaK = R\(B'*P*S*C'-B'*Aci'*(P+H'*V*H)*xb*yb'+B'*Aci'*H'*V*r0*yb')/(C*S*C')-Kold;
        deltaK = R\(B'*P*S*C'-B'*Aci'*(P)*xb*yb')/(C*S*C')-Kold;
        % choose alpha s.t. Acl_new is stable
        Knew = Kold + alpha*deltaK;
        stab = isstable(ss(A-B*Knew*C,G-B*Knew*F,C,D));
        if stab == 1
            if (abs(Jnew-Jold) >= 0.001)
                Jold = Jnew;
                Kold = Knew;
            else
                K = Knew;
                break;
            end 
        else
            alpha = alpha - 0.01;
        end
    end
end