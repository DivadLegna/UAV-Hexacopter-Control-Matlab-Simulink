function A = calcularParametrosAerodinamicos(P)
% ============================================================
% CALCULARPARAMETROSAERODINAMICOS
%
% Parámetros aerodinámicos obtenidos mediante
% SOLIDWORKS Flow Simulation.
%
% Configuración CFD:
%   UAV con aspas estacionarias
%   Velocidad de viento = 20 km/h
%
% Correspondencia de ejes:
%
%   SOLIDWORKS      SIMULINK
%       X      -->      X
%       Z      -->      Y
%       Y      -->      Z
%
% ============================================================

arguments
    P struct
end

%% Condiciones atmosféricas utilizadas en SolidWorks
A.pressureCFD    = 101325;     % [Pa]
A.temperatureCFD = 293.2;      % [K]
A.Rair           = 287.05;     % [J/(kg*K)]

% Densidad correspondiente exactamente a las condiciones CFD.
A.rhoCFD = A.pressureCFD / ...
           (A.Rair*A.temperatureCFD);

%% Velocidad utilizada en las simulaciones CFD

A.windSpeedCFD = 20/3.6;       % [m/s]

%% Fuerzas obtenidas mediante CFD
%
% Se utilizan los valores PROMEDIO de los Surface Goals.
%
% Configuración:
%   con aspas
%   malla global nivel 7
%   malla local refinada

% ------------------------------------------------------------
% Dirección X
%
% SolidWorks X = Simulink X
% ------------------------------------------------------------

A.FxCFD = 1.435;               % [N]

% ------------------------------------------------------------
% Dirección Y
%
% SolidWorks Z = Simulink Y
% ------------------------------------------------------------

A.FyCFD = 1.026;               % [N]

% ------------------------------------------------------------
% Dirección Z
%
% SolidWorks Y = Simulink Z
%
% Pendiente de realizar CFD con:
% Vy_SOLIDWORKS = 5.556 m/s
% ------------------------------------------------------------

A.FzCFD = 1.147;    % [N]
%% Producto Cd*A
%
% Fuerza aerodinámica:
%
% Fd = 0.5*rho*Cd*A*V^2
%
% Por tanto:
%
% CdA = 2*Fd/(rho*V^2)

A.CdA_x = 2*A.FxCFD / ...
    (A.rhoCFD*A.windSpeedCFD^2);

A.CdA_y = 2*A.FyCFD / ...
    (A.rhoCFD*A.windSpeedCFD^2);

if ~isnan(A.FzCFD)

    A.CdA_z = 2*A.FzCFD / ...
        (A.rhoCFD*A.windSpeedCFD^2);

else

    A.CdA_z = NaN;

end

%% Coeficientes cuadráticos para Simulink
%
% Durante la simulación se utilizará la densidad atmosférica
% de operación almacenada en P.rho.
%
% La expresión implementada será:
%
% Fd = -kD * Vrel * abs(Vrel)
%
% donde:
%
% kD = 0.5*rho*CdA

A.kDx = 0.5*P.rho*A.CdA_x;
A.kDy = 0.5*P.rho*A.CdA_y;

if ~isnan(A.CdA_z)

    A.kDz = 0.5*P.rho*A.CdA_z;

else

    A.kDz = 0;

end

%% Velocidad máxima de viento definida en los requerimientos

A.windSpeedMax = 20/3.6;       % [m/s]

%% Mostrar resultados

fprintf('\n======= PARAMETROS AERODINAMICOS CFD =======\n');

fprintf('rho CFD      = %.4f kg/m^3\n', ...
    A.rhoCFD);

fprintf('Velocidad CFD = %.3f m/s\n', ...
    A.windSpeedCFD);

fprintf('\n');

fprintf('CdA_x = %.5f m^2\n', ...
    A.CdA_x);

fprintf('CdA_y = %.5f m^2\n', ...
    A.CdA_y);

if ~isnan(A.CdA_z)

    fprintf('CdA_z = %.5f m^2\n', ...
        A.CdA_z);

else

    fprintf('CdA_z = pendiente de CFD vertical\n');

end

fprintf('\n');

fprintf('kDx = %.5f kg/m\n', ...
    A.kDx);

fprintf('kDy = %.5f kg/m\n', ...
    A.kDy);

if ~isnan(A.kDz)

    fprintf('kDz = %.5f kg/m\n', ...
        A.kDz);

else

    fprintf('kDz = pendiente de CFD vertical\n');

end

fprintf('============================================\n\n');

end