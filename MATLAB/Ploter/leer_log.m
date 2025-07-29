function [t, Y] = leer_log(nombre_archivo)
% Lee CSV con: timestamp completo, dato1, dato2, ...
% Devuelve:
% - t: tiempo relativo en segundos
% - Y: matriz de datos numéricos

    fid = fopen(nombre_archivo, 'r');
    fgetl(fid); % Saltar encabezado

    t = [];
    Y = [];
    tiempo0 = [];
    columnas_esperadas = [];

    while ~feof(fid)
        linea = strtrim(fgetl(fid));
        if isempty(linea), continue; end

        partes = strsplit(linea, ',');
        if numel(partes) < 2, continue; end

        try
            timestamp = datetime(partes{1}, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
        catch
            continue
        end

        datos = str2double(partes(2:end));
        if any(isnan(datos))
            continue
        end

        if isempty(columnas_esperadas)
            columnas_esperadas = numel(datos);
        elseif numel(datos) ~= columnas_esperadas
            continue
        end

        if isempty(tiempo0)
            tiempo0 = timestamp;
        end

        t(end+1,1) = seconds(timestamp - tiempo0);
        Y(end+1,:) = datos;
    end

    fclose(fid);
end

