function K = SAS_RollYaw_LQR(A,B,silent)
    K0 = SAS_RollYaw_EigAsgn(A,B,silent);
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
    C = eye(6); D = zeros(6,2);
    if silent == 0
        sys0 = ss(A-B*K0*C,B,C,D);
        damp(sys0)
    end
    Q = diag([35 5 5 35 0 0]);
    R = 10*eye(2);
    K = LQR(A,B,C,D,Q,R,K0);
    if silent == 0
        sys1 = ss(Ac,B,C,D);
        eig(sys1);
        damp(sys1);
    end
 end 