function plot_rls_first_second(port_name, baud_rate)
%PLOT_RLS_FIRST_SECOND Captura y grafica el primer segundo de datos RLS.
%
% Uso:
%   plot_rls_first_second                 % COM3, 115200 baudios
%   plot_rls_first_second("COM3")
%   plot_rls_first_second("COM3", 115200)

    if nargin < 2
        baud_rate = 115200;
    end
    if nargin < 1 || strlength(string(port_name)) == 0
        port_name = "COM3";
    end

    serial_device = serialport(port_name, baud_rate);
    configureTerminator(serial_device, "LF");
    serial_device.Timeout = 2;
    flush(serial_device);
    cleanup = onCleanup(@() close_serial(serial_device)); %#ok<NASGU>

    time_data = [];
    measured_data = [];
    predicted_data = [];
    parameter_a_data = [];
    parameter_b_data = [];
    capture_started = false;

    fprintf("Esperando el primer dato RLS válido en %s...\n", string(port_name));

    while true
        try
            serial_line = strtrim(readline(serial_device));
        catch exception
            if strcmp(exception.identifier, "MATLAB:serialport:ReadTimeout")
                continue;
            end
            rethrow(exception);
        end

        values = sscanf(char(serial_line), ...
            "RLS: y=%f, y'=%f, a=%f, b=%f");

        if numel(values) ~= 4 || any(~isfinite(values))
            continue;
        end

        if ~capture_started
            capture_timer = tic;
            capture_started = true;
            elapsed_time = 0.0;
            fprintf("Primer dato recibido. Capturando durante un segundo...\n");
        else
            elapsed_time = toc(capture_timer);
        end

        if elapsed_time > 1.0
            break;
        end

        time_data(end + 1) = elapsed_time; %#ok<AGROW>
        measured_data(end + 1) = values(1); %#ok<AGROW>
        predicted_data(end + 1) = values(2); %#ok<AGROW>
        parameter_a_data(end + 1) = values(3); %#ok<AGROW>
        parameter_b_data(end + 1) = values(4); %#ok<AGROW>
    end

    figure_handle = figure( ...
        "Name", "Primer segundo de identificación RLS", ...
        "NumberTitle", "off", ...
        "Color", "w");

    output_axes = subplot(2, 1, 1, "Parent", figure_handle);
    plot(output_axes, time_data, measured_data, ...
        "LineWidth", 1.5, "DisplayName", "y medida");
    hold(output_axes, "on");
    plot(output_axes, time_data, predicted_data, "--", ...
        "LineWidth", 1.5, "DisplayName", "y' predicha");
    grid(output_axes, "on");
    xlim(output_axes, [0, 1]);
    xlabel(output_axes, "Tiempo desde el primer dato [s]");
    ylabel(output_axes, "Velocidad [rad/s]");
    title(output_axes, "Salida medida y predicción");
    legend(output_axes, "Location", "best");

    parameter_axes = subplot(2, 1, 2, "Parent", figure_handle);
    plot(parameter_axes, time_data, parameter_a_data, ...
        "LineWidth", 1.5, "DisplayName", "a");
    hold(parameter_axes, "on");
    plot(parameter_axes, time_data, parameter_b_data, ...
        "LineWidth", 1.5, "DisplayName", "b");
    grid(parameter_axes, "on");
    xlim(parameter_axes, [0, 1]);
    xlabel(parameter_axes, "Tiempo desde el primer dato [s]");
    ylabel(parameter_axes, "Valor estimado");
    title(parameter_axes, "Parámetros identificados");
    legend(parameter_axes, "Location", "best");

    fprintf("Captura terminada: %d muestras válidas entre 0 y 1 segundo.\n", ...
        numel(time_data));
end

function close_serial(serial_device)
% Libera COM3 al terminar, incluso si se produce un error.
    if ~isempty(serial_device) && isvalid(serial_device)
        flush(serial_device);
        delete(serial_device);
    end
end
