% Constant Values
g = 9.81;
pi = 3.14159265359;

% Mass of the rocket (body)
M_b = 0.543;   

% Distance from the Cg to test the periodic oscillation
C_s = 0.3;

%  Length of the string tied to find Cg
L_s = 0.65;

% Moment arm distance - i.e distance between TVC mount & Cg
D_m = 0.28;

% Period of oscillation - Done 10 oscillation
T_s = 16.03;
TAvg_s = 1.063;

% Mass Moment of Inertia calculation
MMOI = (M_b * g * TAvg_s^2 * C_s^2) / (4^2 * pi^2 * L_s);