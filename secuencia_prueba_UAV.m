% SECUENCIA_PRUEBA_UAV Referencias para comprobar movimiento en Simulink/RViz.
% Ejecutar DESPUES de Inicializacion desde la Command Window de MATLAB.
% No usa clear ni modifica masa, inercias, ganancias o parametros de ROS 2.
% Supuestos: posiciones en metros; yaw en radianes; estado inicial en reposo
% en x=y=z=0, yaw=0. Prueba visual inicial con wind_enable=0 en el MODELO.
% Los tiempos son ventanas de observacion, no tiempos de establecimiento
% validados. Termina a 1 m: no representa una prueba de contacto/aterrizaje.
%
% Conectar cuatro bloques From Workspace a las referencias x, y, z y yaw:
% Data: ref_x / ref_y / ref_z / ref_psi; Sample time: 0;
% Interpolate data: desactivado; despues del ultimo dato: Holding final value.
% Conservar los ZOH de la publicacion ROS y los lazos de control existentes.
% Stop time: Tfin_prueba; Simulation Pacing: 1x.
% Si InitFcn ejecuta un script que borra el workspace, ejecutar esta secuencia
% al final de esa inicializacion para recrear las referencias antes de simular.

% Columnas: tiempo [s], x [m], y [m], z [m], yaw [rad].
secuencia_UAV = [
     0, 0, 0, 0, 0;
     3, 0, 0, 3, 0;       % Ascenso
    15, 3, 0, 3, 0;       % X positivo
    25, 0, 0, 3, 0;       % Regreso en X
    35, 0, 3, 3, 0;       % Y positivo
    45, 0, 0, 3, 0;       % Regreso en Y
    55, 0, 0, 3, pi/2;    % Yaw +90 grados
    65, 0, 0, 3, 0;       % Recuperar yaw inicial
    75, 0, 0, 1, 0;       % Descenso hasta 1 m
    85, 0, 0, 1, 0        % Fin, mantener referencia
];

assert(all(diff(secuencia_UAV(:,1)) > 0), ...
    'Los tiempos deben ser estrictamente crecientes.');
assert(all(isfinite(secuencia_UAV(:))), 'Hay valores no finitos.');

ref_x = timeseries(secuencia_UAV(:,2), secuencia_UAV(:,1), 'Name', 'x_d');
ref_y = timeseries(secuencia_UAV(:,3), secuencia_UAV(:,1), 'Name', 'y_d');
ref_z = timeseries(secuencia_UAV(:,4), secuencia_UAV(:,1), 'Name', 'z_d');
ref_psi = timeseries(secuencia_UAV(:,5), secuencia_UAV(:,1), 'Name', 'psi_d');

ref_x = setinterpmethod(ref_x, 'zoh');
ref_y = setinterpmethod(ref_y, 'zoh');
ref_z = setinterpmethod(ref_z, 'zoh');
ref_psi = setinterpmethod(ref_psi, 'zoh');
ref_x.DataInfo.Units = 'm';
ref_y.DataInfo.Units = 'm';
ref_z.DataInfo.Units = 'm';
ref_psi.DataInfo.Units = 'rad';
ref_x.TimeInfo.Units = 'seconds';
ref_y.TimeInfo.Units = 'seconds';
ref_z.TimeInfo.Units = 'seconds';
ref_psi.TimeInfo.Units = 'seconds';
Tfin_prueba = secuencia_UAV(end,1);

disp(array2table(secuencia_UAV, 'VariableNames', ...
    {'t_s','x_m','y_m','z_m','yaw_rad'}));
fprintf('Referencias listas. Stop time = Tfin_prueba (%.0f s).\n', Tfin_prueba);
