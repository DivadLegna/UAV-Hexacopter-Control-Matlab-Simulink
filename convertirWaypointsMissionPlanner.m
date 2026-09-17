function [referencia, mision] = convertirWaypointsMissionPlanner(archivo, velocidad, velocidadYaw)
%CONVERTIRWAYPOINTSMISSIONPLANNER Convierte QGC WPL 110 a una secuencia local.
%
% [referencia, mision] = convertirWaypointsMissionPlanner(archivo)
% [referencia, mision] = convertirWaypointsMissionPlanner(archivo, velocidad)
% [referencia, mision] = convertirWaypointsMissionPlanner(archivo, velocidad, velocidadYaw)
%
% Entradas
%   archivo       Ruta del archivo .waypoints generado por Mission Planner.
%   velocidad     Rapidez traslacional constante [m/s]. Valor predeterminado: 5.
%   velocidadYaw  Rapidez angular constante [deg/s]. Valor predeterminado: 45.
%
% Salidas
%   referencia    Matriz para el bloque From Workspace:
%                 [tiempo_s, X_Este_m, Y_Norte_m, Z_Arriba_m, yaw_rad]
%   mision        Tabla con los eventos convertidos y el comando MAVLink.
%
% Comandos procesados
%   16  MAV_CMD_NAV_WAYPOINT
%   20  MAV_CMD_NAV_RETURN_TO_LAUNCH
%   22  MAV_CMD_NAV_TAKEOFF
%   115 MAV_CMD_CONDITION_YAW
%
% El RTL se aproxima mediante un retorno horizontal al origen a la altitud
% actual, seguido de un descenso vertical. El comportamiento real depende de
% los parametros RTL_ALT y RTL_ALT_FINAL configurados en ArduPilot.

    arguments
        archivo (1,1) string
        velocidad (1,1) double {mustBePositive} = 5
        velocidadYaw (1,1) double {mustBePositive} = 45
    end

    fid = fopen(archivo, 'r');
    assert(fid ~= -1, 'No se pudo abrir el archivo: %s', archivo);
    cierre = onCleanup(@() fclose(fid)); %#ok<NASGU>

    encabezado = strtrim(fgetl(fid));
    assert(strcmp(encabezado, 'QGC WPL 110'), ...
        'El archivo no tiene el encabezado QGC WPL 110 esperado.');

    datos = textscan(fid, repmat('%f', 1, 12), ...
        'Delimiter', {sprintf('\t'), ' '}, 'MultipleDelimsAsOne', true, ...
        'CollectOutput', true);
    datos = datos{1};
    assert(size(datos, 2) == 12 && ~isempty(datos), ...
        'No se encontraron registros validos en el archivo.');

    % Columnas QGC WPL 110
    secuenciaOriginal = datos(:, 1);
    frame             = datos(:, 3);
    comando           = datos(:, 4);
    param1            = datos(:, 5);
    latitud           = datos(:, 9);
    longitud          = datos(:, 10);
    altitud           = datos(:, 11);

    % La fila marcada como actual (current = 1) se usa como origen local.
    idxOrigen = find(datos(:, 2) == 1, 1, 'first');
    if isempty(idxOrigen)
        idxOrigen = find(latitud ~= 0 & longitud ~= 0, 1, 'first');
    end
    assert(~isempty(idxOrigen), 'No fue posible determinar el origen geografico.');

    lat0 = latitud(idxOrigen);
    lon0 = longitud(idxOrigen);

    % Aproximacion local equirectangular, adecuada para una mision pequena.
    radioTierra = 6378137; % [m], radio ecuatorial WGS84
    xEste  = radioTierra .* deg2rad(longitud - lon0) .* cosd(lat0);
    yNorte = radioTierra .* deg2rad(latitud - lat0);

    t = 0;
    posicion = [0, 0, 0];
    yawContinuoDeg = 0;

    tiempoSalida = 0;
    posicionSalida = posicion;
    yawSalida = deg2rad(yawContinuoDeg);
    comandoSalida = 0;
    indiceSalida = secuenciaOriginal(idxOrigen);

    for k = 1:size(datos, 1)
        cmd = comando(k);

        switch cmd
            case 22 % TAKEOFF
                % Mission Planner puede escribir latitud y longitud como cero.
                destino = [0, 0, altitud(k)];
                agregarTraslacion(destino, cmd, secuenciaOriginal(k));

            case 16 % NAV_WAYPOINT
                destino = [xEste(k), yNorte(k), altitud(k)];
                agregarTraslacion(destino, cmd, secuenciaOriginal(k));

            case 115 % CONDITION_YAW
                yawObjetivo = mod(param1(k), 360);
                deltaYaw = mod(yawObjetivo - yawContinuoDeg + 180, 360) - 180;
                dt = abs(deltaYaw) / velocidadYaw;

                if dt > 0
                    t = t + dt;
                    yawContinuoDeg = yawContinuoDeg + deltaYaw;
                    agregarMuestra(cmd, secuenciaOriginal(k));
                end

            case 20 % RETURN_TO_LAUNCH
                % Aproximacion: regreso horizontal y descenso vertical.
                destino = [0, 0, posicion(3)];
                agregarTraslacion(destino, cmd, secuenciaOriginal(k));
                destino = [0, 0, 0];
                agregarTraslacion(destino, cmd, secuenciaOriginal(k));

            otherwise
                % Los comandos no contemplados se conservan fuera de la
                % referencia, ya que no definen X, Y, Z o yaw.
        end
    end

    referencia = [tiempoSalida, posicionSalida, yawSalida];

    mision = table(indiceSalida, comandoSalida, tiempoSalida, ...
        posicionSalida(:,1), posicionSalida(:,2), posicionSalida(:,3), yawSalida, ...
        'VariableNames', {'IndiceWPL', 'Comando', 'Tiempo_s', ...
        'X_Este_m', 'Y_Norte_m', 'Z_Arriba_m', 'Yaw_rad'});

    fprintf('Origen: %.7f deg, %.7f deg\n', lat0, lon0);
    fprintf('Duracion estimada: %.2f s\n', referencia(end,1));
    fprintf('Muestras generadas: %d\n', size(referencia,1));

    function agregarTraslacion(destino, cmdActual, indiceActual)
        distancia = norm(destino - posicion);
        if distancia <= 1e-9
            return
        end

        t = t + distancia / velocidad;
        posicion = destino;
        agregarMuestra(cmdActual, indiceActual);
    end

    function agregarMuestra(cmdActual, indiceActual)
        tiempoSalida(end+1,1) = t;
        posicionSalida(end+1,:) = posicion;
        yawSalida(end+1,1) = deg2rad(yawContinuoDeg);
        comandoSalida(end+1,1) = cmdActual;
        indiceSalida(end+1,1) = indiceActual;
    end
end
