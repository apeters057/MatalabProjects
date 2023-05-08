
%% Your name and design parameters 

StudentName = 'Anthony Peters';

KI = 916e10;  % Integral gain
KP = 1000e4;   % Proportional gain

NotchD = [17    25    20    12    11    10];     % Depth of the notch filters
NotchW = [5800  5000  5000  9000  7700  5800];   % Width of the notch filters
NotchF = [6900  7300  11000 12000 14300 13600];  % Frequencies of the notch filters

%% Controller Calculation 
Ts = 1/40000;                              % Sampling time
PI_s = tf([KP KI],[1 0]);                  % PI controller
PI_z = c2d(PI_s,Ts,'matched');             % PI controller discretization
C_z = PI_z;                                % Full controller initilization
for i = 1:max(6,length(NotchD))            % Maximum 6 notch filters are allowed!
    C_z = C_z * NotchTF(NotchD(i),NotchW(i),NotchF(i),NotchF(i), Ts);
end