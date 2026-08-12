function plot_rls_serial(port_name, baud_rate, csv_filename)
%PLOT_RLS_SERIAL Grafica en tiempo real los datos RLS enviados por el ESP32.
%
% Uso:
%   plot_rls_serial                 % usa COM3 a 115200 baudios
%   plot_rls_serial("COM3")
%   plot_rls_serial("COM3", 115200)
%   plot_rls_serial("COM3", 115200, "mis_datos_rls.csv")
%
% El programa espera líneas con el formato:
%   RLS: y=12.345000, y'=12.100000, a=0.950000, b=0.123000

    if nargin < 2
        baud_rate = 115200;
    end

    if nargin < 1 || strlength(string(port_name)) == 0
        port_name = "COM3";
    end

    if nargin < 3 || strlength(string(csv_filename)) == 0
        timestamp = datestr(now, "yyyymmdd_HHMMSS");
        csv_filename = fullfile(pwd, ...
            sprintf("rls_data_%s.csv", timestamp));
    end

    csv_file = fopen(csv_filename, "w");
    if csv_file == -1
        error("plot_rls_serial:CsvOpenFailed", ...
            "No se pudo crear el archivo CSV: %s", string(csv_filename));
    end
    csv_cleanup = onCleanup(@() close_csv(csv_file));
    fprintf(csv_file, ...
        ['sample,time_s,measured_speed_rad_s,predicted_speed_rad_s,' ...
         'prediction_error_rad_s,a,b\n']);

    serial_device = serialport(port_name, baud_rate);
    configureTerminator(serial_device, "LF");
    serial_device.Timeout = 2;
    flush(serial_device);
    cleanup = onCleanup(@() close_serial(serial_device));

    figure_handle = figure( ...
        "Name", "Identificación RLS del motor", ...
        "NumberTitle", "off", ...
        "Color", "w");

    output_axes = subplot(2, 1, 1, "Parent", figure_handle);
    measured_line = animatedline(output_axes, ...
        "Color", [0.00, 0.45, 0.74], "LineWidth", 1.5, ...
        "DisplayName", "y medida");
    predicted_line = animatedline(output_axes, ...
        "Color", [0.85, 0.33, 0.10], "LineWidth", 1.5, ...
        "LineStyle", "--", "DisplayName", "y' predicha");
    grid(output_axes, "on");
    xlabel(output_axes, "Tiempo [s]");
    ylabel(output_axes, "Velocidad [rad/s]");
    title(output_axes, "Salida medida y predicción RLS");
    legend(output_axes, "Location", "best");
    xlim(output_axes, [0, 1]);

    parameter_axes = subplot(2, 1, 2, "Parent", figure_handle);
    parameter_a_line = animatedline(parameter_axes, ...
        "Color", [0.47, 0.67, 0.19], "LineWidth", 1.5, ...
        "DisplayName", "a");
    parameter_b_line = animatedline(parameter_axes, ...
        "Color", [0.49, 0.18, 0.56], "LineWidth", 1.5, ...
        "DisplayName", "b");
    grid(parameter_axes, "on");
    xlabel(parameter_axes, "Tiempo [s]");
    ylabel(parameter_axes, "Valor estimado");
    title(parameter_axes, "Parámetros identificados");
    legend(parameter_axes, "Location", "best");
    xlim(parameter_axes, [0, 1]);

    linkaxes([output_axes, parameter_axes], "x");
    start_time = tic;
    sample_count = 0;

    fprintf("Escuchando %s a %d baudios. Cierra la figura para terminar.\n", ...
        string(port_name), baud_rate);
    fprintf("Guardando datos RLS en: %s\n", string(csv_filename));

    while isvalid(figure_handle)
        try
            serial_line = strtrim(readline(serial_device));
        catch exception
            if strcmp(exception.identifier, "MATLAB:serialport:ReadTimeout")
                drawnow limitrate;
                continue;
            end
            rethrow(exception);
        end

        values = sscanf(char(serial_line), ...
            "RLS: y=%f, y'=%f, a=%f, b=%f");

        if numel(values) ~= 4 || any(~isfinite(values))
            continue;
        end

        elapsed_time = toc(start_time);
        sample_count = sample_count + 1;
        prediction_error = values(1) - values(2);

        fprintf(csv_file, "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", ...
            sample_count, elapsed_time, values(1), values(2), ...
            prediction_error, values(3), values(4));

        addpoints(measured_line, elapsed_time, values(1));
        addpoints(predicted_line, elapsed_time, values(2));
        addpoints(parameter_a_line, elapsed_time, values(3));
        addpoints(parameter_b_line, elapsed_time, values(4));

        % Conserva toda la trayectoria: el límite izquierdo permanece en
        % cero y el derecho crece con el tiempo total de adquisición.
        trajectory_end = max(1.0, elapsed_time * 1.02);
        xlim(output_axes, [0, trajectory_end]);
        xlim(parameter_axes, [0, trajectory_end]);

        if mod(sample_count, 5) == 0
            drawnow limitrate;
        end
    end

    fprintf("Captura finalizada: %d muestras guardadas en %s\n", ...
        sample_count, string(csv_filename));
end

function close_serial(serial_device)
% Libera el puerto al terminar o al producirse un error.
    if ~isempty(serial_device) && isvalid(serial_device)
        flush(serial_device);
        delete(serial_device);
    end
end

function close_csv(csv_file)
% Cierra el CSV para escribir todos los datos pendientes.
    if csv_file ~= -1
        fclose(csv_file);
    end
end
