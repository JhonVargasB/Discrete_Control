function plot_lqi_serial(port_name, baud_rate)
%PLOT_LQI_SERIAL Grafica en tiempo real la prueba del controlador LQI.
%
% Uso:
%   plot_lqi_serial
%   plot_lqi_serial("COM3", 115200)

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
    cleanup = onCleanup(@() close_serial(serial_device));

    figure_handle = figure( ...
        "Name", "Prueba LQI de posicion", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "Position", [100, 50, 1100, 850]);

    position_axes = subplot(5, 1, 1, "Parent", figure_handle);
    reference_line = animatedline(position_axes, ...
        "Color", [0.85, 0.33, 0.10], "LineStyle", "--", ...
        "LineWidth", 1.4, "DisplayName", "Referencia");
    position_line = animatedline(position_axes, ...
        "Color", [0.00, 0.45, 0.74], "LineWidth", 1.4, ...
        "DisplayName", "Posicion");
    grid(position_axes, "on");
    ylabel(position_axes, "Posicion [rad]");
    title(position_axes, "Seguimiento de posicion");
    legend(position_axes, "Location", "best");

    speed_axes = subplot(5, 1, 2, "Parent", figure_handle);
    delta_speed_line = animatedline(speed_axes, ...
        "Color", [0.47, 0.67, 0.19], "LineStyle", ":", ...
        "LineWidth", 1.1, "DisplayName", "omega delta");
    filtered_speed_line = animatedline(speed_axes, ...
        "Color", [0.49, 0.18, 0.56], "LineWidth", 1.3, ...
        "DisplayName", "omega filtrada");
    grid(speed_axes, "on");
    ylabel(speed_axes, "Velocidad [rad/s]");
    title(speed_axes, "Estado de velocidad");
    legend(speed_axes, "Location", "best");

    error_axes = subplot(5, 1, 3, "Parent", figure_handle);
    yyaxis(error_axes, "left");
    error_line = animatedline(error_axes, ...
        "Color", [0.85, 0.33, 0.10], "LineWidth", 1.2, ...
        "DisplayName", "Error");
    ylabel(error_axes, "Error [rad]");
    yyaxis(error_axes, "right");
    integral_line = animatedline(error_axes, ...
        "Color", [0.49, 0.18, 0.56], "LineWidth", 1.2, ...
        "DisplayName", "v");
    ylabel(error_axes, "Estado integral v");
    grid(error_axes, "on");
    title(error_axes, "Error y estado integral");
    legend(error_axes, "Location", "best");

    control_axes = subplot(5, 1, 4, "Parent", figure_handle);
    raw_control_line = animatedline(control_axes, ...
        "Color", [0.64, 0.08, 0.18], "LineStyle", "--", ...
        "LineWidth", 1.1, "DisplayName", "u raw [%]");
    saturated_control_line = animatedline(control_axes, ...
        "Color", [0.47, 0.67, 0.19], "LineWidth", 1.2, ...
        "DisplayName", "u saturado [%]");
    applied_pwm_line = animatedline(control_axes, ...
        "Color", [0.00, 0.45, 0.74], "LineWidth", 1.3, ...
        "DisplayName", "PWM aplicado [%]");
    yline(control_axes, 100, ":r", "Saturacion +100%");
    yline(control_axes, -100, ":r", "Saturacion -100%");
    grid(control_axes, "on");
    ylabel(control_axes, "Control [%]");
    title(control_axes, "Accion de control");
    legend(control_axes, "Location", "best");

    flag_axes = subplot(5, 1, 5, "Parent", figure_handle);
    anti_windup_flag_line = animatedline(flag_axes, ...
        "Color", [0.64, 0.08, 0.18], "LineWidth", 1.2, ...
        "DisplayName", "Anti-windup activo");
    position_reached_line = animatedline(flag_axes, ...
        "Color", [0.00, 0.45, 0.74], "LineStyle", "--", ...
        "LineWidth", 1.2, "DisplayName", "Posicion aceptada");
    grid(flag_axes, "on");
    ylim(flag_axes, [-0.1, 1.1]);
    yticks(flag_axes, [0, 1]);
    xlabel(flag_axes, "Tiempo [s]");
    ylabel(flag_axes, "Estado logico");
    title(flag_axes, "Anti-windup y banda de posicion");
    legend(flag_axes, "Location", "best");

    linkaxes([position_axes, speed_axes, error_axes, control_axes, flag_axes], ...
        "x");
    sample_count = 0;

    fprintf("Escuchando prueba LQI en %s a %d baudios.\n", ...
        string(port_name), baud_rate);

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
            ['LQI: %f,%f,%f,%f,%f,%f,' ...
             '%f,%f,%f,%f,%f,%f']);
        if numel(values) ~= 12 || any(~isfinite(values))
            continue;
        end

        time_s = values(1);
        reference = values(2);
        theta = values(3);
        omega_delta = values(4);
        omega_filtered = values(5);
        position_error = values(6);
        integral_state = values(7);
        raw_control_percent = 100.0 * values(8);
        saturated_control_percent = 100.0 * values(9);
        pwm_percent = values(10);
        anti_windup_flag = values(11);
        position_reached_flag = values(12);

        addpoints(reference_line, time_s, reference);
        addpoints(position_line, time_s, theta);
        addpoints(delta_speed_line, time_s, omega_delta);
        addpoints(filtered_speed_line, time_s, omega_filtered);
        addpoints(error_line, time_s, position_error);
        addpoints(integral_line, time_s, integral_state);
        addpoints(raw_control_line, time_s, raw_control_percent);
        addpoints(saturated_control_line, time_s, ...
            saturated_control_percent);
        addpoints(applied_pwm_line, time_s, pwm_percent);
        addpoints(anti_windup_flag_line, time_s, anti_windup_flag);
        addpoints(position_reached_line, time_s, position_reached_flag);

        trajectory_end = max(4.0, time_s * 1.02);
        xlim(position_axes, [0, trajectory_end]);

        sample_count = sample_count + 1;
        if mod(sample_count, 5) == 0
            drawnow limitrate;
        end
    end
end

function close_serial(serial_device)
    if ~isempty(serial_device) && isvalid(serial_device)
        flush(serial_device);
        delete(serial_device);
    end
end
