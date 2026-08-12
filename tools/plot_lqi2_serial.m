function plot_lqi2_serial(port_name, baud_rate)
%PLOT_LQI2_SERIAL Grafica en tiempo real los dos controladores LQI.
%
% Uso:
%   plot_lqi2_serial
%   plot_lqi2_serial("COM3", 115200)
%
% Formato esperado:
%   LQI1: t,r,theta,omega_medida,omega_filtrada,e,v,u_prop,u,pwm,aw,ok
%   LQI2: t,r,theta,omega_medida,omega_filtrada,e,v,u_prop,u,pwm,aw,ok

    if nargin < 2
        baud_rate = 115200;
    end
    if nargin < 1 || strlength(string(port_name)) == 0
        port_name = "COM3";
    end

    display_window_s = 30.0;
    maximum_points = 3000;

    serial_device = serialport(port_name, baud_rate);
    configureTerminator(serial_device, "LF");
    serial_device.Timeout = 2;
    flush(serial_device);
    cleanup = onCleanup(@() close_serial(serial_device));

    figure_handle = figure( ...
        "Name", "Control LQI de dos motores", ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "Position", [40, 40, 1450, 900]);

    layout = tiledlayout(figure_handle, 5, 2, ...
        "TileSpacing", "compact", ...
        "Padding", "compact");
    title(layout, "Telemetria LQI en tiempo real");

    motor_colors = [
        0.00, 0.45, 0.74;
        0.85, 0.33, 0.10;
    ];

    plots(1) = create_motor_plots(layout, 1, motor_colors(1, :), ...
        maximum_points);
    plots(2) = create_motor_plots(layout, 2, motor_colors(2, :), ...
        maximum_points);

    all_axes = [plots(1).axes, plots(2).axes];
    linkaxes(all_axes, "x");

    latest_time = 0.0;
    sample_count = 0;

    fprintf("Escuchando LQI1 y LQI2 en %s a %d baudios.\n", ...
        string(port_name), baud_rate);
    fprintf("Cierra la figura para terminar.\n");

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

        tokens = regexp(char(serial_line), ...
            '^LQI([12]):\s*(.*)$', "tokens", "once");
        if isempty(tokens)
            continue;
        end

        motor_id = str2double(tokens{1});
        values = str2double(split(string(tokens{2}), ","));
        values = values(:);

        if numel(values) ~= 12 || any(~isfinite(values))
            continue;
        end

        time_s = values(1);
        reference = values(2);
        theta = values(3);
        omega_measured = values(4);
        omega_filtered = values(5);
        position_error = values(6);
        integral_state = values(7);
        proposed_control_percent = 100.0 * values(8);
        requested_control_percent = 100.0 * values(9);
        pwm_percent = values(10);
        anti_windup_flag = values(11);
        position_reached_flag = values(12);

        motor_plot = plots(motor_id);

        addpoints(motor_plot.reference, time_s, reference);
        addpoints(motor_plot.theta, time_s, theta);
        addpoints(motor_plot.omega_measured, time_s, omega_measured);
        addpoints(motor_plot.omega_filtered, time_s, omega_filtered);
        addpoints(motor_plot.error, time_s, position_error);
        addpoints(motor_plot.integral, time_s, integral_state);
        addpoints(motor_plot.u_proposed, time_s, ...
            proposed_control_percent);
        addpoints(motor_plot.u_requested, time_s, ...
            requested_control_percent);
        addpoints(motor_plot.pwm, time_s, pwm_percent);
        addpoints(motor_plot.anti_windup, time_s, anti_windup_flag);
        addpoints(motor_plot.position_reached, time_s, ...
            position_reached_flag);

        latest_time = max(latest_time, time_s);
        x_start = max(0.0, latest_time - display_window_s);
        x_end = max(display_window_s, latest_time);
        xlim(plots(1).axes(1), [x_start, x_end]);

        sample_count = sample_count + 1;
        if mod(sample_count, 2) == 0
            drawnow limitrate;
        end
    end
end

function motor_plot = create_motor_plots(layout, motor_id, color, maximum_points)
    tile_column = motor_id;

    position_axes = nexttile(layout, tile_column);
    reference_line = animatedline(position_axes, ...
        "Color", color, "LineStyle", "--", "LineWidth", 1.2, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Referencia");
    theta_line = animatedline(position_axes, ...
        "Color", color, "LineWidth", 1.5, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Theta");
    configure_axes(position_axes, motor_id, ...
        "Seguimiento de posicion", "Posicion [rad]");
    legend(position_axes, "Location", "best");

    speed_axes = nexttile(layout, tile_column + 2);
    omega_measured_line = animatedline(speed_axes, ...
        "Color", color, "LineStyle", ":", "LineWidth", 1.0, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Omega medida");
    omega_filtered_line = animatedline(speed_axes, ...
        "Color", color, "LineWidth", 1.4, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Omega filtrada");
    configure_axes(speed_axes, motor_id, ...
        "Velocidad angular", "Velocidad [rad/s]");
    legend(speed_axes, "Location", "best");

    error_axes = nexttile(layout, tile_column + 4);
    yyaxis(error_axes, "left");
    error_line = animatedline(error_axes, ...
        "Color", color, "LineWidth", 1.3, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Error");
    ylabel(error_axes, "Error [rad]");
    yyaxis(error_axes, "right");
    integral_line = animatedline(error_axes, ...
        "Color", 0.55 * color, "LineStyle", "--", "LineWidth", 1.2, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Integral v");
    ylabel(error_axes, "Estado integral v");
    grid(error_axes, "on");
    title(error_axes, sprintf("Motor %d - Error e integral", motor_id));
    legend(error_axes, "Location", "best");

    control_axes = nexttile(layout, tile_column + 6);
    u_proposed_line = animatedline(control_axes, ...
        "Color", 0.55 * color, "LineStyle", ":", "LineWidth", 1.0, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "u propuesta");
    u_requested_line = animatedline(control_axes, ...
        "Color", color, "LineStyle", "--", "LineWidth", 1.2, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "u solicitada");
    pwm_line = animatedline(control_axes, ...
        "Color", color, "LineWidth", 1.5, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "PWM aplicado");
    configure_axes(control_axes, motor_id, ...
        "Accion de control", "Control [%]");
    legend(control_axes, "Location", "best");

    flag_axes = nexttile(layout, tile_column + 8);
    anti_windup_line = animatedline(flag_axes, ...
        "Color", color, "LineWidth", 1.2, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Anti-windup");
    position_reached_line = animatedline(flag_axes, ...
        "Color", 0.55 * color, "LineStyle", "--", "LineWidth", 1.2, ...
        "MaximumNumPoints", maximum_points, ...
        "DisplayName", "Posicion alcanzada");
    configure_axes(flag_axes, motor_id, ...
        "Estados logicos", "Estado");
    ylim(flag_axes, [-0.1, 1.1]);
    yticks(flag_axes, [0, 1]);
    xlabel(flag_axes, "Tiempo [s]");
    legend(flag_axes, "Location", "best");

    motor_plot.reference = reference_line;
    motor_plot.theta = theta_line;
    motor_plot.omega_measured = omega_measured_line;
    motor_plot.omega_filtered = omega_filtered_line;
    motor_plot.error = error_line;
    motor_plot.integral = integral_line;
    motor_plot.u_proposed = u_proposed_line;
    motor_plot.u_requested = u_requested_line;
    motor_plot.pwm = pwm_line;
    motor_plot.anti_windup = anti_windup_line;
    motor_plot.position_reached = position_reached_line;
    motor_plot.axes = [position_axes, speed_axes, error_axes, ...
        control_axes, flag_axes];
end

function configure_axes(axes_handle, motor_id, axes_title, y_label)
    grid(axes_handle, "on");
    ylabel(axes_handle, y_label);
    title(axes_handle, sprintf("Motor %d - %s", motor_id, axes_title));
end

function close_serial(serial_device)
    if ~isempty(serial_device) && isvalid(serial_device)
        flush(serial_device);
        delete(serial_device);
    end
end
