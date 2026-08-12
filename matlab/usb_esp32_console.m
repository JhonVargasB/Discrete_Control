%% Consola USB para cinematica inversa/directa y telemetria del ESP32
% CI px py           : coordenadas en centimetros
% CD theta1 theta2   : angulos en grados

portName = "COM3";
baudRate = 115200;

fprintf("Conectando al ESP32 en %s...\n", portName);

try
    esp32 = serialport(portName, baudRate, "Timeout", 1);
catch exception
    error("No se pudo abrir %s. Cierre otros monitores seriales.\n%s", ...
          portName, exception.message);
end

cleanupObject = onCleanup(@() closeEsp32Port(esp32));

configureTerminator(esp32, "CR/LF");

figureHandle = figure("Name", "Brazo 2R - ESP32", ...
                      "NumberTitle", "off");

angleAxes = subplot(2, 1, 1, "Parent", figureHandle);
hold(angleAxes, "on");
theta1MeasuredLine = animatedline(angleAxes, "Color", [0 0.4470 0.7410], ...
                                  "LineWidth", 1.5, "DisplayName", "theta1");
theta1ReferenceLine = animatedline(angleAxes, "Color", [0 0.4470 0.7410], ...
                                   "LineStyle", "--", "DisplayName", "theta1 ref");
theta2MeasuredLine = animatedline(angleAxes, "Color", [0.8500 0.3250 0.0980], ...
                                  "LineWidth", 1.5, "DisplayName", "theta2");
theta2ReferenceLine = animatedline(angleAxes, "Color", [0.8500 0.3250 0.0980], ...
                                   "LineStyle", "--", "DisplayName", "theta2 ref");
grid(angleAxes, "on");
xlabel(angleAxes, "Muestra");
ylabel(angleAxes, "Angulo (rad)");
title(angleAxes, "Seguimiento articular");
legend(angleAxes, "Location", "best");

armAxes = subplot(2, 1, 2, "Parent", figureHandle);
hold(armAxes, "on");
trajectoryLine = animatedline(armAxes, "Color", [0 0.4470 0.7410], ...
                              "LineWidth", 1.2, ...
                              "DisplayName", "Trayectoria medida");
measuredPoint = plot(armAxes, 0, 0, "bo", "MarkerSize", 8, ...
                     "MarkerFaceColor", "b", ...
                     "DisplayName", "Posicion medida");
targetPoint = plot(armAxes, 0, 0, "rx", "MarkerSize", 12, ...
                   "LineWidth", 2, "DisplayName", "Objetivo");
grid(armAxes, "on");
axis(armAxes, "equal");
xlim(armAxes, [-20 20]);
ylim(armAxes, [-20 20]);
xlabel(armAxes, "px (cm)");
ylabel(armAxes, "py (cm)");
title(armAxes, "Posicion XY");
legend(armAxes, "Location", "best");

esp32.UserData = struct( ...
    "sample", 0, ...
    "theta1MeasuredLine", theta1MeasuredLine, ...
    "theta1ReferenceLine", theta1ReferenceLine, ...
    "theta2MeasuredLine", theta2MeasuredLine, ...
    "theta2ReferenceLine", theta2ReferenceLine, ...
    "armAxes", armAxes, ...
    "trajectoryLine", trajectoryLine, ...
    "measuredPoint", measuredPoint, ...
    "targetPoint", targetPoint);

configureCallback(esp32, "terminator", @showEsp32Line);

fprintf("Conectado. Se mostraran logs y telemetria del ESP32.\n");
fprintf("CI px py         -> cinematica inversa en centimetros.\n");
fprintf("CD theta1 theta2 -> cinematica directa en grados.\n");
fprintf("Ejemplos: 'CI 13 13' o 'CD 30 15'.\n");
fprintf("Escriba 'q' para cerrar.\n\n");

while true
    commandText = strtrim(input("comando > ", "s"));

    if strcmpi(commandText, "q")
        break;
    end

    normalizedText = strrep(commandText, ",", " ");
    tokens = split(string(strtrim(normalizedText)));
    tokens(tokens == "") = [];

    if numel(tokens) ~= 3
        fprintf("Entrada invalida. Use 'CI px py' o 'CD theta1 theta2'.\n");
        continue;
    end

    mode = upper(tokens(1));
    values = str2double(tokens(2:3));

    if (mode ~= "CI" && mode ~= "CD") || any(~isfinite(values))
        fprintf("Entrada invalida. Use 'CI px py' o 'CD theta1 theta2'.\n");
        continue;
    end

    if mode == "CD" && any(abs(values) > 90)
        fprintf("Los angulos de CD deben estar entre -90 y 90 grados.\n");
        continue;
    end

    transmittedLine = sprintf("%s %.6f %.6f", ...
                              char(mode), values(1), values(2));

    writeline(esp32, transmittedLine);
    fprintf("MATLAB -> ESP32: %s\\r\\n\n", transmittedLine);
end

fprintf("Consola finalizada.\n");

function showEsp32Line(source, ~)
    try
        receivedLine = readline(source);
        fprintf("ESP32 -> MATLAB: %s\n", receivedLine);

        fields = split(strtrim(string(receivedLine)), ",");
        if numel(fields) ~= 10 || fields(1) ~= "ARM"
            return;
        end

        mode = fields(2);
        values = str2double(fields(3:10));
        if any(~isfinite(values))
            return;
        end

        pxReference = values(1);
        pyReference = values(2);
        pxMeasured = values(3);
        pyMeasured = values(4);
        theta1Reference = values(5);
        theta2Reference = values(6);
        theta1 = values(7);
        theta2 = values(8);

        state = source.UserData;
        state.sample = state.sample + 1;

        addpoints(state.theta1MeasuredLine, state.sample, theta1);
        addpoints(state.theta1ReferenceLine, state.sample, theta1Reference);
        addpoints(state.theta2MeasuredLine, state.sample, theta2);
        addpoints(state.theta2ReferenceLine, state.sample, theta2Reference);

        addpoints(state.trajectoryLine, pxMeasured, pyMeasured);
        set(state.measuredPoint, "XData", pxMeasured, "YData", pyMeasured);
        set(state.targetPoint, "XData", pxReference, "YData", pyReference);
        title(state.armAxes, "Posicion XY - modo " + mode);

        source.UserData = state;
        drawnow limitrate nocallbacks;
    catch exception
        fprintf(2, "Error leyendo el ESP32: %s\n", exception.message);
    end
end

function closeEsp32Port(port)
    configureCallback(port, "off");
    flush(port);
end
