function P = parametrosHexacoptero()
% ============================================================
% PARAMETROSHEXACOPTERO
% Hexacóptero para fotogrametría
% Autor: Roa Trejo Angel David
% ============================================================

%% Constantes
P.g = 9.80665;                 % [m/s^2]

%% Atmósfera
% rhoRef representa provisionalmente las condiciones
% de la tabla del fabricante.

P.rhoRef = 1.225;              % [kg/m^3]
P.rho    = 0.98;               % [kg/m^3]

%% Propiedades generales
P.mass         = 6.36386;      % [kg]
P.numberMotors = 6;
P.armLength    = 0.425;        % [m]

%% Geometría
P.armDiameter  = 0.020;        % [m]
P.propDiameter = 15*0.0254;    % [m]
P.propRadius   = P.propDiameter/2;
P.propMass     = 0.040;        % [kg]

%% Momentos de inercia obtenidos de SolidWorks
JcadX = 0.20092663733;         % [kg*m^2]
JcadY = 0.24753190714;         % [kg*m^2]
JcadZ = 0.17443826024;         % [kg*m^2]

% Selección del eje vertical del CAD:
P.verticalAxisCAD = "Y";

switch P.verticalAxisCAD
    case "Y"
        % Cuerpo [x y z] = CAD [X Z Y]
        % z del cuerpo es el eje vertical/yaw.
        P.Jxx = JcadX;
        P.Jyy = JcadZ;
        P.Jzz = JcadY;

    case "Z"
        % Cuerpo [x y z] = CAD [X Y Z]
        P.Jxx = JcadX;
        P.Jyy = JcadY;
        P.Jzz = JcadZ;

    otherwise
        error('Eje vertical CAD no reconocido.');
end

P.J = diag([P.Jxx, P.Jyy, P.Jzz]);

%% Momento de inercia del conjunto rotor-hélice
P.JrMotor = 1.176495e-5;       % [kg*m^2]

% Aproximación inicial de la hélice mediante dos palas radiales.
P.JrProp = (1/3)*P.propMass*P.propRadius^2;

P.Jr = P.JrMotor + P.JrProp;

%% Coeficientes de arrastre provisionales
% Se conservarán temporalmente para poder simular.
% Después se sustituirán por los valores calculados.

P.kftx = 0.10;                 % [N*s/m]
P.kfty = 0.10;                 % [N*s/m]
P.kftz = 0.15;                 % [N*s/m]

P.kfax = 0.10;                 % Provisional
P.kfay = 0.10;                 % Provisional
P.kfaz = 0.15;                 % Provisional

%% Sistema propulsor
P.rotor = calcularParametrosRotor(P);

end