function K = calcularGananciasControl(P)
% ============================================================
% CALCULARGANANCIASCONTROL
% Ganancias PD obtenidas por asignación de polos de segundo orden
% ============================================================

%% Especificaciones temporales
spec.phi.ts   = 0.5;
spec.theta.ts = spec.phi.ts;
spec.psi.ts   = 2*spec.phi.ts;

spec.z.ts = 2.5*spec.phi.ts;
spec.x.ts = 5*spec.phi.ts;
spec.y.ts = spec.x.ts;

spec.phi.Mp   = 0.05;
spec.theta.Mp = spec.phi.Mp;
spec.psi.Mp   = spec.phi.Mp;

spec.x.Mp = 0.001;
spec.y.Mp = 0.001;
spec.z.Mp = 0.001;

%% Parámetros de segundo orden
[K.phi.zeta,   K.phi.wn]   = segundoOrden(spec.phi.Mp, spec.phi.ts);
[K.theta.zeta, K.theta.wn] = segundoOrden(spec.theta.Mp, spec.theta.ts);
[K.psi.zeta,   K.psi.wn]   = segundoOrden(spec.psi.Mp, spec.psi.ts);

[K.x.zeta, K.x.wn] = segundoOrden(spec.x.Mp, spec.x.ts);
[K.y.zeta, K.y.wn] = segundoOrden(spec.y.Mp, spec.y.ts);
[K.z.zeta, K.z.wn] = segundoOrden(spec.z.Mp, spec.z.ts);

%% Control de actitud
% La salida de cada controlador debe estar en N*m.

K.phi.Kp = P.Jxx*K.phi.wn^2;
K.phi.Ki = 0;
K.phi.Kd = 2*K.phi.zeta*K.phi.wn*P.Jxx;

K.theta.Kp = P.Jyy*K.theta.wn^2;
K.theta.Ki = 0;
K.theta.Kd = 2*K.theta.zeta*K.theta.wn*P.Jyy;

K.psi.Kp = P.Jzz*K.psi.wn^2;
K.psi.Ki = 0;
K.psi.Kd = 2*K.psi.zeta*K.psi.wn*P.Jzz;

%% Control de posición horizontal
% Las salidas son referencias angulares aproximadas.

K.x.Kp = K.x.wn^2/P.g;
K.x.Ki = 0;
K.x.Kd = (2*K.x.zeta*K.x.wn - P.kftx/P.mass)/P.g;

K.y.Kp = K.y.wn^2/P.g;
K.y.Ki = 0;
K.y.Kd = (2*K.y.zeta*K.y.wn - P.kfty/P.mass)/P.g;

%% Altitud: salida expresada como fuerza [N]
K.zForce.Kp = P.mass*K.z.wn^2;
K.zForce.Ki = 0;
K.zForce.Kd = 2*P.mass*K.z.zeta*K.z.wn - P.kftz;

%% Altitud: salida normalizada como aceleración [m/s^2]
% Esta forma corresponde a tu script anterior.

K.zAccel.Kp = K.z.wn^2;
K.zAccel.Ki = 0;
K.zAccel.Kd = 2*K.z.zeta*K.z.wn - P.kftz/P.mass;

%% Equilibrio
K.Thover = P.mass*P.g;

fprintf('\n=============== GANANCIAS PD ===============\n');
fprintf('Roll  : Kp = %.6f, Kd = %.6f\n', ...
    K.phi.Kp, K.phi.Kd);

fprintf('Pitch : Kp = %.6f, Kd = %.6f\n', ...
    K.theta.Kp, K.theta.Kd);

fprintf('Yaw   : Kp = %.6f, Kd = %.6f\n', ...
    K.psi.Kp, K.psi.Kd);

fprintf('X     : Kp = %.6f, Kd = %.6f\n', ...
    K.x.Kp, K.x.Kd);

fprintf('Y     : Kp = %.6f, Kd = %.6f\n', ...
    K.y.Kp, K.y.Kd);

fprintf('Z fuerza: Kp = %.6f, Kd = %.6f\n', ...
    K.zForce.Kp, K.zForce.Kd);

fprintf('Z aceleracion: Kp = %.6f, Kd = %.6f\n', ...
    K.zAccel.Kp, K.zAccel.Kd);

fprintf('Thover = %.3f N\n', K.Thover);
fprintf('============================================\n\n');

end

function [zeta, wn] = segundoOrden(Mp, ts)

validateattributes(Mp, {'numeric'}, ...
    {'scalar','positive','<',1});

validateattributes(ts, {'numeric'}, ...
    {'scalar','positive'});

logMp = log(Mp);

zeta = -logMp/sqrt(pi^2 + logMp^2);
wn   = 4/(zeta*ts);

end