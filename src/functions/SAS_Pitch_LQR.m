function K = SAS_Pitch_LQR(A,B,silent)
    K0 = SAS_Pitch_EigAsgn(A,B,silent);
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
    C = eye(4); D = zeros(4,1);
    if silent == 0
        sys0 = ss(A-B*K0*C,B,C,D);
        isstable(sys0)
        damp(sys0)
    end
    Q = diag([35 5 5 0]); R = 10;
    K = LQR(A,B,C,D,Q,R,K0);
    if silent == 0
        sys1 = ss(Ac,B,C,D);
        eig(sys1);
        damp(sys1);
    end
end