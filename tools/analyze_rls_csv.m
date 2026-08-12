function result = analyze_rls_csv(csv_filename, stability_window_s, relative_tolerance)
%ANALYZE_RLS_CSV Grafica una captura RLS y estima a y b estabilizados.
%
% Uso:
%   result = analyze_rls_csv
%   result = analyze_rls_csv("rls_data_20260730_051234.csv")
%   result = analyze_rls_csv("datos.csv", 2.0, 0.02)
%
% stability_window_s: ventana usada para verificar estabilidad [s].
% relative_tolerance: desviacion estandar relativa maxima permitida.

    if nargin < 3 || isempty(relative_tolerance)
        relative_tolerance = 0.02;
    end
    if nargin < 2 || isempty(stability_window_s)
        stability_window_s = 2.0;
    end
    if nargin < 1 || strlength(string(csv_filename)) == 0
        [file_name, folder_name] = uigetfile("*.csv", ...
            "Selecciona una captura RLS");
        if isequal(file_name, 0)
            result = [];
            return;
        end
        csv_filename = fullfile(folder_name, file_name);
    end

    validateattributes(stability_window_s, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'});
    validateattributes(relative_tolerance, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive', '<', 1});

    data = readtable(csv_filename);
    required_columns = { ...
        'time_s', ...
        'measured_speed_rad_s', ...
        'predicted_speed_rad_s', ...
        'prediction_error_rad_s', ...
        'a', ...
        'b'};

    missing_columns = setdiff(required_columns, data.Properties.VariableNames);
    if ~isempty(missing_columns)
        error('analyze_rls_csv:MissingColumns', ...
            'Faltan columnas requeridas en el CSV: %s', ...
            strjoin(missing_columns, ', '));
    end

    finite_rows = isfinite(data.time_s) & ...
        isfinite(data.measured_speed_rad_s) & ...
        isfinite(data.predicted_speed_rad_s) & ...
        isfinite(data.prediction_error_rad_s) & ...
        isfinite(data.a) & isfinite(data.b);
    data = data(finite_rows, :);
    data = sortrows(data, 'time_s');

    if height(data) < 5
        error('analyze_rls_csv:InsufficientData', ...
            'El CSV debe contener al menos cinco muestras validas.');
    end

    time_s = data.time_s;
    parameter_a = data.a;
    parameter_b = data.b;

    positive_periods = diff(time_s);
    positive_periods = positive_periods(positive_periods > 0);
    if isempty(positive_periods)
        error('analyze_rls_csv:InvalidTime', ...
            'La columna time_s no contiene tiempos crecientes.');
    end

    average_receive_interval_s = ...
        (time_s(end) - time_s(1)) / (height(data) - 1);
    stable_start_index = find( ...
        time_s >= time_s(end) - stability_window_s, 1, 'first');
    stable_indices = stable_start_index:height(data);
    actual_window_s = time_s(end) - time_s(stable_start_index);

    if actual_window_s < 0.9 * stability_window_s || ...
            numel(stable_indices) < 5
        error('analyze_rls_csv:ShortCapture', ...
            'La captura dura muy poco para una ventana de %.3f s.', ...
            stability_window_s);
    end

    final_a = mean(parameter_a(stable_indices));
    final_b = mean(parameter_b(stable_indices));
    final_a_std = std(parameter_a(stable_indices));
    final_b_std = std(parameter_b(stable_indices));

    magnitude_floor = 1e-3;
    absolute_tolerance = 1e-6;
    limit_a = absolute_tolerance + relative_tolerance * ...
        max(abs(final_a), magnitude_floor);
    limit_b = absolute_tolerance + relative_tolerance * ...
        max(abs(final_b), magnitude_floor);
    parameters_are_stable = final_a_std <= limit_a && ...
        final_b_std <= limit_b;

    result = struct( ...
        'stabilized', false, ...
        'a', NaN, ...
        'b', NaN, ...
        'a_std', NaN, ...
        'b_std', NaN, ...
        'stable_start_time_s', NaN, ...
        'average_receive_interval_s', average_receive_interval_s, ...
        'window_s', stability_window_s, ...
        'window_sample_count', numel(stable_indices), ...
        'relative_tolerance', relative_tolerance);

    if parameters_are_stable
        result.stabilized = true;
        result.a = final_a;
        result.b = final_b;
        result.a_std = final_a_std;
        result.b_std = final_b_std;
        result.stable_start_time_s = time_s(stable_start_index);
    end

    figure_handle = figure( ...
        'Name', 'Analisis de identificacion RLS', ...
        'NumberTitle', 'off', ...
        'Color', 'w');

    output_axes = subplot(3, 1, 1, 'Parent', figure_handle);
    plot(output_axes, time_s, data.measured_speed_rad_s, ...
        'LineWidth', 1.2, 'DisplayName', 'Velocidad medida');
    hold(output_axes, 'on');
    plot(output_axes, time_s, data.predicted_speed_rad_s, '--', ...
        'LineWidth', 1.2, 'DisplayName', 'Velocidad predicha');
    grid(output_axes, 'on');
    ylabel(output_axes, 'Velocidad [rad/s]');
    title(output_axes, 'Salida medida y prediccion RLS');
    legend(output_axes, 'Location', 'best');

    parameter_axes = subplot(3, 1, 2, 'Parent', figure_handle);
    plot(parameter_axes, time_s, parameter_a, ...
        'LineWidth', 1.2, 'DisplayName', 'a');
    hold(parameter_axes, 'on');
    plot(parameter_axes, time_s, parameter_b, ...
        'LineWidth', 1.2, 'DisplayName', 'b');
    grid(parameter_axes, 'on');
    ylabel(parameter_axes, 'Parametro');
    title(parameter_axes, 'Convergencia de parametros');
    legend(parameter_axes, 'Location', 'best');

    error_axes = subplot(3, 1, 3, 'Parent', figure_handle);
    plot(error_axes, time_s, data.prediction_error_rad_s, ...
        'Color', [0.49, 0.18, 0.56], 'LineWidth', 1.1);
    grid(error_axes, 'on');
    xlabel(error_axes, 'Tiempo [s]');
    ylabel(error_axes, 'Error [rad/s]');
    title(error_axes, 'Error de prediccion');

    linkaxes([output_axes, parameter_axes, error_axes], 'x');
    xlim(output_axes, [time_s(1), time_s(end)]);

    if result.stabilized
        xline(output_axes, result.stable_start_time_s, ':k', ...
            'Inicio estable', 'LabelOrientation', 'horizontal');
        xline(parameter_axes, result.stable_start_time_s, ':k', ...
            'Inicio estable', 'LabelOrientation', 'horizontal');
        xline(error_axes, result.stable_start_time_s, ':k', ...
            'Inicio estable', 'LabelOrientation', 'horizontal');

        fprintf('\nParametros RLS estabilizados desde t = %.3f s:\n', ...
            result.stable_start_time_s);
        fprintf('  a = %.9f   (desviacion estandar %.3e)\n', ...
            result.a, result.a_std);
        fprintf('  b = %.9f   (desviacion estandar %.3e)\n', ...
            result.b, result.b_std);
    else
        fprintf('\nNo se detecto estabilidad al final de la captura.\n');
        fprintf(['Prueba con una captura mas larga o aumenta la tolerancia, ' ...
                 'por ejemplo: analyze_rls_csv("%s", %.1f, 0.02)\n'], ...
            string(csv_filename), stability_window_s);
    end
end
