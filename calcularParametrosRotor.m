function R = calcularParametrosRotor(P)
% ============================================================
% CALCULARPARAMETROSROTOR
% Estimación provisional para:
% Motor D4114 400 KV
% Hélice 15x5.5 CF
% Batería 6S
%
% La velocidad se estima mediante una curva experimental
% cercana de un motor 4014 400 KV con hélice 16x5.4 CF.
% ============================================================

arguments
    P struct
end

%% Datos exactos del conjunto seleccionado
R.KV             = 400;               % [rpm/V]
R.voltage        = 22.2;              % [V]
R.currentFull    = 22.6;              % [A]
R.powerElecFull  = 502;               % [W]
R.thrustFullRef  = 2.780 * P.g;       % [N]
R.propDiameter   = 15 * 0.0254;       % [m]
R.propPitch      = 5.5 * 0.0254;      % [m]

%% Curva experimental de referencia
proxy.D           = 16 * 0.0254;      % [m]
proxy.thrustFull   = 2.890 * P.g;      % [N]
proxy.rpmFull      = 6700;             % [rpm]
proxy.rpm80        = 6206;             % [rpm]
proxy.thrust80     = 2.480 * P.g;      % [N]
proxy.powerFull    = 488;              % [W]

%% Estimación de RPM máximas
% T = CT*rho*n^2*D^4
% Se supone CT aproximadamente igual entre ambas hélices.

R.rpmFullRef = proxy.rpmFull ...
    * sqrt(R.thrustFullRef / proxy.thrustFull) ...
    * (proxy.D / R.propDiameter)^2;

R.omegaFullRef = 2*pi*R.rpmFullRef/60;

%% Coeficiente de empuje a densidad de referencia
R.bRef = R.thrustFullRef / R.omegaFullRef^2;

%% Corrección por densidad del aire
% b es aproximadamente proporcional a rho.

R.b = R.bRef * P.rho / P.rhoRef;

%% Estimación del coeficiente de par
% La potencia publicada es eléctrica, no mecánica.
% etaShaft representa eficiencia conjunta motor + ESC.

R.etaShaft = 0.85;

R.powerShaftFull = R.etaShaft * R.powerElecFull;
R.torqueFullRef  = R.powerShaftFull / R.omegaFullRef;

R.dRef = R.torqueFullRef / R.omegaFullRef^2;
R.d    = R.dRef * P.rho / P.rhoRef;

%% Coeficientes adimensionales
nFull = R.rpmFullRef/60;

R.CT = R.thrustFullRef / ...
    (P.rhoRef*nFull^2*R.propDiameter^4);

R.CP = R.powerShaftFull / ...
    (P.rhoRef*nFull^3*R.propDiameter^5);

%% Límite operacional del 80 %
rpmRatio80 = proxy.rpm80/proxy.rpmFull;

R.rpmMax       = rpmRatio80 * R.rpmFullRef;
R.omegaMax     = 2*pi*R.rpmMax/60;
R.thrustMax    = R.b * R.omegaMax^2;
R.thrustMax_g  = 1000*R.thrustMax/P.g;

%% Condición de hover
R.thrustHover = P.mass*P.g/P.numberMotors;
R.omegaHover  = sqrt(R.thrustHover/R.b);
R.rpmHover    = 60*R.omegaHover/(2*pi);

%% Márgenes
R.totalThrustMax = P.numberMotors*R.thrustMax;
R.thrustToWeight = R.totalThrustMax/(P.mass*P.g);

%% Relación torque-empuje para el mixer
R.kappa = R.d/R.b;

%% Verificaciones
assert(R.rpmHover < R.rpmMax, ...
    'No existe margen suficiente entre hover y RPM máximas.');

fprintf('\n============= SISTEMA PROPULSOR =============\n');
fprintf('RPM máximas de referencia : %.0f rpm\n', R.rpmFullRef);
fprintf('RPM máximas al 80 %%       : %.0f rpm\n', R.rpmMax);
fprintf('RPM de hover               : %.0f rpm\n', R.rpmHover);
fprintf('b a densidad operativa     : %.4e N*s^2\n', R.b);
fprintf('d a densidad operativa     : %.4e N*m*s^2\n', R.d);
fprintf('CT estimado                : %.4f\n', R.CT);
fprintf('CP estimado                : %.4f\n', R.CP);
fprintf('Empuje por rotor al 80 %%   : %.1f g\n', R.thrustMax_g);
fprintf('Relacion empuje/peso       : %.2f\n', R.thrustToWeight);
fprintf('=============================================\n\n');

end