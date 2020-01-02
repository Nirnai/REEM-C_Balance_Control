dt = 0.001;
zc = 0.83;
g = 9.81;


%% Jerk

% A = [ 
%       1 dt dt^2/2 ;
%       0  1     dt ;
%       0  0      1 
%      ];
%  
%  B = [ dt^3/6; dt^2/2; dt ];

A = [ 
      0  1  0;
      0  0  1;
      0  0  0 
     ];
 
 B = [ 0; 1; 0 ];
 
 Q = eye(3);
 Q(1,1) = 10;
 Q(2,2) = 1;
 Q(3,3) = 20;
 R = 1;

 K = lqr(A, B, Q, R);

%% Acceleration
A = [ 
    -1/dt 1/dt 0 ;
         0   0 1 ;
         0   0 0 
     ];
 
 B = [ -zc/(g*dt); 0; 1 ];
 
 
 % LQR
 
 Q = eye(3);
 Q(1,1) = 1000;
 Q(2,2) = 50;
 Q(3,3) = 10;
 R = 0.1;
 
 K = lqr(A,B,Q,R);
 
 
 %% Ankle Strategy
 
 M = 67.5523;
 zc = 0.82;
 g = 9.81;
 w = sqrt(g/zc);
 
 A = [0    1 ;
      w^2  0];
 
 B = [0; w^2/(M*g)];
 
 Q = eye(2);
 Q(1,1) = 1;
 Q(2,2) = 1;
 R = 0.1;
 
 K = lqr(A,B,Q,R);