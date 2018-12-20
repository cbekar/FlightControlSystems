function [kqLUT,kpLUT,krLUT] = getGainLUTs()
global S
    for i=1:26
        A = S.linear(i).a; B = S.linear(i).b;
        % Swap states and inputs for the ease of generating A B matrices.
        Along = [A(1,1) A(1,2) A(1,5) A(1,8);
                A(2,1) A(2,2) A(2,5) A(2,8);
                A(5,1) A(5,2) A(5,5) A(5,8);
                A(8,1) A(8,2) A(8,5) A(8,8)
               ];
        Alat  = [A(3,3) A(3,4) A(3,7) A(3,9);
                A(4,3) A(4,4) A(4,7) A(4,9);
                A(7,3) A(7,4) A(7,7) A(7,9);
                A(9,3) A(9,4) A(9,7) A(9,9)
               ];
        Blong = [B(1,1:2); B(2,1:2); B(5,1:2); B(8,1:2)];
        Blat  = [B(3,3:4); B(4,3:4); B(7,3:4); B(9,3:4)];
        Clong = [0 1 0 0; 0 0 0 1]; Clat = [0 0 1 0; 0 0 0 1];
        D  = zeros(2);
        % 1) Find SAS gains
        % 1-A) Longitudinal
        % Pitch Damper
        Bpd = Blong(:,2);
        Apd_aug = [Along, -Bpd, zeros(4,1); [0 0 0 0 -10 0]; [0 10 0 0 0 -10]];
        Bpd_aug = [zeros(4,1); 10; 0];
        Cpd_aug = [Clong zeros(2); zeros(1,5) 1];
        k= logspace(-2,1,2000);
        % Here we can observe performance degredation for alpha feedback
        rlocus(Apd_aug,Bpd_aug,Cpd_aug(3,:),0,k); 
        grid on
        axis([-15,1,-10,10])

        Apd_aug = [Along, -Bpd, zeros(4,1); [0 0 0 0 -10 0]; [0 0 0 10 0 -10]];
        Bpd_aug = [zeros(4,1); 10; 0];
        Cpd_aug = [Clong zeros(2); zeros(1,5) 1];
        figure
        rlocus(Apd_aug,Bpd_aug,Cpd_aug(3,:),0,k); 
        grid on
        axis([-15,1,-10,10])
        % Here we can observe flying quality for kq pitch rate feedback gives good
        % longitudinal frequencies and damping ratios (according to Nelson pg. 167)
        % [pitchSASnum,pitchSASden] = ss2tf(Along,Blong,C,D,2);
        % alpElv = tf(pitchSASnum(1,:),pitchSASden);
        % rlocus(alpElv)
        % qElv   = tf(pitchSASnum(2,:),pitchSASden);
        % rlocus(qElv)
        S.kq(i) = 0.88;
        Apd_cl = Apd_aug - Bpd_aug.*S.kq(i)*Cpd_aug(3,:);
        rlocus(Apd_cl,Bpd_aug,Cpd_aug(3,:),0,k);
        grid on
        axis([-15,1,-10,10])
        %zpk(ss(Apd_cl,Bpd_aug,Cpd_aug(3,:),0))
        t= [0:.02:10]; % 501 points for plot
        %u= [zeros(1,50),1.8*ones(1,451)]'; %step
        u= [-1.8*ones(1,51),1.8*ones(1,50),zeros(1,400)]'; % Doublet
        [y,x]= lsim(Apd_cl,Bpd_aug(:,1),Cpd_aug(3,:),0,u,t); % Linear simulation
        plot(t,y,t,u)
        grid on
        hold on
        [y,x]= lsim(Apd_aug,Bpd_aug(:,1),Cpd_aug(3,:),0,u,t); % Linear simulation
        plot(t,y)
        %% 
        % 1-B) Lateral
        % Roll-Yaw Damper
        Aact = [-10 0; 0 -10]; Bact = [10 0; 0 10]; Cact = [1 0; 0 -1]; 
        actuator = ss(Aact, Bact, Cact, D);
        plant    = ss(Alat, Blat, Clat, D);
        sys1     = series(actuator, plant);
        tauWash = 1;
        aw   = [-1/tauWash]; bw= [0 1/tauWash];  % ?w to be defined
        cw   = [0; -1]; dw= [1 0; 0 1];           % y1=p y2=washed-r
        wash = ss(aw,bw,cw,dw);
        sys2= series(sys1,wash); % x1=wash, x2=beta,.., x6=ail, x7=rdr

        [a,b,c,d]= ssdata(sys2);
        k= linspace(0,.9,3000);
        rlocus(a,b(:,1),c(1,:),0,k); %aileron to roll
        grid on
        axis([-12,1,-5,5])

        t= [0:.02:10]; % 501 points for plot
        u= [zeros(1,50),1.8*ones(1,451)]'; %step
        %u= [-1.8*ones(1,51),1.8*ones(1,50),zeros(1,400)]'; % Doublet
        [y,x]= lsim(a,b(:,1),c(1,:),0,u,t); % Linear simulation
        plot(t,y,t,u)
        grid on

        S.kp(i) = 0.35;
        acl1= a - b(:,1)*S.kp(i)*c(1,:); % Close roll loop
        zpk(ss(acl1,b(:,2),c(2,:),0)); % Yaw tr. fn. + wash
        rlocus( acl1,b(:,2),c(2,:),0,k);
        grid on

        S.kr(i) = 0.6;
        acl2= a - b*[S.kp(i) 0; 0 S.kr(i)]*c;
        zpk(ss(acl2,b(:,2),c(2,:),0)); % c.l. roll-rate t.f.
        rlocus(acl2,b(:,2),c(2,:),0)
        grid on

        t= [0:.02:10]; % 501 points for plot
        u= [-1.8*ones(1,51),1.8*ones(1,50),zeros(1,400)]'; % Doublet
        [y,x]= lsim(acl2,b(:,1),c(1,:),0,u,t); % Linear simulation
        plot(t,y,t,u)
        grid on
        hold on
        [y,x]= lsim(a,b(:,1),c(1,:),0,u,t); % Linear simulation
        plot(t,y,t,u)
        close all
    end
    S.kqLUT(:,1) = S.M';
    S.kqLUT(:,2) = [172 151 159 154 160 92 91 75 76 80 80 68 66 65 73 71 74 78 84 85 91 85 105 104 120 130]/100';
end