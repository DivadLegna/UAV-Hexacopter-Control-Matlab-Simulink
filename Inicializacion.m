%% ==========================================================
% INICIALIZACIÓN DE LA SIMULACIÓN
%% ==========================================================

clear;
close all;
clc;

P = parametrosHexacoptero();
K = calcularGananciasControl(P);

%% Variables físicas compatibles con el modelo existente
g  = P.g;
m  = P.mass;
l  = P.armLength;

Jxx = P.Jxx;
Jyy = P.Jyy;
Jzz = P.Jzz;
Jr  = P.Jr;

OHMr = 0;

kftx = P.kftx;
kfty = P.kfty;
kftz = P.kftz;

kfax = P.kfax;
kfay = P.kfay;
kfaz = P.kfaz;

%% Coeficientes del rotor
b = P.rotor.b;
d = P.rotor.d;

omega_hover = P.rotor.omegaHover;
omega_max   = P.rotor.omegaMax;

rpm_hover = P.rotor.rpmHover;
rpm_max   = P.rotor.rpmMax;

%% Ganancias de actitud
kpphi = K.phi.Kp;
kiphi = K.phi.Ki;
kdphi = K.phi.Kd;

kptheta = K.theta.Kp;
kitheta = K.theta.Ki;
kdtheta = K.theta.Kd;

kppsi = K.psi.Kp;
kipsi = K.psi.Ki;
kdpsi = K.psi.Kd;

%% Posición
kpx = K.x.Kp;
kix = K.x.Ki;
kdx = K.x.Kd;

kpy = K.y.Kp;
kiy = K.y.Ki;
kdy = K.y.Kd;

%% Altitud

kpz = K.zAccel.Kp;
kiz = K.zAccel.Ki;
kdz = K.zAccel.Kd;

Thover = K.Thover;

%% Factores de paso
u1 = 1;
u2 = 1;
u3 = 1;
u4 = 1;

%% Referencias iniciales
setpoint_Phi   = 0;
setpoint_Theta = 0;
setpoint_Psi   = 0;
setpoint_z     = 0;
setpoint_x     = 0;
setpoint_y     = 0;

%% Sistema de Chen

chen_a = 35;
chen_b = 3;
chen_c = 28;

%% Secuencia para roll
tiempo_phi = [0; 2; 4; 6; 8; 10; 12];

referencia_phi = [
     0;
     pi/10;
    -pi/10;
     pi/20;
    -pi/20;
     pi/10;
     0
];

setpoint_phi_sequence = [tiempo_phi referencia_phi];

%% Secuencia para pitch
tiempo_theta = [0; 2; 4; 6; 8; 10; 12];

referencia_theta = [
     0;
     pi/10;
    -pi/10;
     pi/20;
    -pi/20;
     pi/10;
     0
];

setpoint_theta_sequence = [tiempo_theta referencia_theta];
tau_motor = 0.05;   % [s], provisional