%% Consola USB para enviar px, py al ESP32 y mostrar todos sus mensajes
% Formato enviado al ESP32: px,py\r\n

%% CONFIGURACIÓN DEL PUERTO SERIAL

% Nombre del puerto COM donde está conectado el ESP32.
portName = "COM3";

% Velocidad de comunicación serial.
baudRate = 115200;

% Informa que se intentará establecer la conexión.
fprintf("Conectando al ESP32 en %s...\n", portName);

%% APERTURA DEL PUERTO SERIAL

try
    % Crea el objeto de comunicación serial.
    % El tiempo máximo de espera para una operación es de 1 segundo.
    esp32 = serialport(portName, baudRate, "Timeout", 1);
catch exception
    % Si el puerto no puede abrirse, detiene el programa y muestra el error.
    % Esto puede ocurrir si otro monitor serial está usando el puerto.
    error("No se pudo abrir %s. Cierre otros monitores seriales.\n%s", ...
        portName, exception.message);
end

% Crea un objeto de limpieza automática.
% Cuando el script termine, se ejecutará closeEsp32Port.
cleanupObject = onCleanup(@() closeEsp32Port(esp32));

% Configura CR/LF como terminador de cada mensaje recibido o enviado.
configureTerminator(esp32, "CR/LF");

%% CREACIÓN DE LA VENTANA GRÁFICA

% Crea la ventana donde se mostrarán los ángulos y el brazo.
figureHandle = figure("Name", "Brazo 2R - ESP32", ...
    "NumberTitle", "off");

%% GRÁFICA DE ÁNGULOS ARTICULARES

% Crea el primer gráfico de la ventana.
angleAxes = subplot(2, 1, 1, "Parent", figureHandle);

% Permite añadir varias curvas en el mismo gráfico.
hold(angleAxes, "on");

% Línea animada del ángulo medido de la primera articulación.
theta1MeasuredLine = animatedline(angleAxes, "Color", [0 0.4470 0.7410], ...
    "LineWidth", 1.5, "DisplayName", "theta1");

% Línea animada de la referencia de la primera articulación.
% Se muestra con línea discontinua.
theta1ReferenceLine = animatedline(angleAxes, "Color", [0 0.4470 0.7410], ...
    "LineStyle", "--", "DisplayName", "theta1 ref");

% Línea animada del ángulo medido de la segunda articulación.
theta2MeasuredLine = animatedline(angleAxes, "Color", [0.8500 0.3250 0.0980], ...
    "LineWidth", 1.5, "DisplayName", "theta2");

% Línea animada de la referencia de la segunda articulación.
theta2ReferenceLine = animatedline(angleAxes, "Color", [0.8500 0.3250 0.0980], ...
    "LineStyle", "--", "DisplayName", "theta2 ref");

% Activa la cuadrícula del gráfico.
grid(angleAxes, "on");

% Etiquetas y título del gráfico.
xlabel(angleAxes, "Muestra");
ylabel(angleAxes, "Angulo (grados)");
title(angleAxes, "Seguimiento articular");

% Muestra la leyenda de las cuatro curvas.
legend(angleAxes, "Location", "best");

%% GRÁFICA DE LA POSICIÓN DEL BRAZO

% Crea el segundo gráfico de la ventana.
armAxes = subplot(2, 1, 2, "Parent", figureHandle);

% Permite añadir varios elementos en el mismo gráfico.
hold(armAxes, "on");

% Crea la línea que representa los dos eslabones del brazo.
% Inicialmente, los tres puntos están ubicados en el origen.
armLine = plot(armAxes, [0 0 0], [0 0 0], "o-", ...
    "LineWidth", 2, "DisplayName", "Brazo medido");

% Crea una marca roja para representar la posición objetivo.
targetPoint = plot(armAxes, 0, 0, "rx", "MarkerSize", 12, ...
    "LineWidth", 2, "DisplayName", "Objetivo");

% Activa la cuadrícula.
grid(armAxes, "on");

% Mantiene la misma escala para los ejes X e Y.
axis(armAxes, "equal");

% Define los límites visibles del espacio de trabajo.
xlim(armAxes, [-20 20]);
ylim(armAxes, [-20 20]);

% Etiquetas y título del gráfico.
xlabel(armAxes, "px (cm)");
ylabel(armAxes, "py (cm)");
title(armAxes, "Posicion XY");

% Muestra la leyenda del brazo y del punto objetivo.
legend(armAxes, "Location", "best");

%% ALMACENAMIENTO DEL ESTADO DE LA INTERFAZ

% Guarda en UserData el contador de muestras y los objetos gráficos.
% Estos datos serán utilizados por la función showEsp32Line.
esp32.UserData = struct( ...
    "sample", 0, ...
    "theta1MeasuredLine", theta1MeasuredLine, ...
    "theta1ReferenceLine", theta1ReferenceLine, ...
    "theta2MeasuredLine", theta2MeasuredLine, ...
    "theta2ReferenceLine", theta2ReferenceLine, ...
    "armLine", armLine, ...
    "targetPoint", targetPoint);

%% CONFIGURACIÓN DE LA RECEPCIÓN SERIAL

% Ejecuta showEsp32Line cada vez que se recibe una línea completa,
% es decir, cuando MATLAB detecta el terminador CR/LF.
configureCallback(esp32, "terminator", @showEsp32Line);

% Muestra las instrucciones de uso de la consola.
fprintf("Conectado. Se mostraran logs y telemetria del ESP32.\n");
fprintf("Escriba dos numeros como '1.25 -0.50' o '1.25,-0.50'.\n");
fprintf("Escriba 'q' para cerrar.\n\n");

%% BUCLE PRINCIPAL PARA ENVIAR COORDENADAS

while true
    % Solicita al usuario las coordenadas px y py como texto.
    commandText = strtrim(input("px py > ", "s"));

    % Finaliza el bucle si el usuario escribe q.
    if strcmpi(commandText, "q")
        break;
    end

    % Reemplaza las comas por espacios para aceptar ambos formatos:
    % "1.25,-0.50" y "1.25 -0.50".
    normalizedText = strrep(commandText, ",", " ");

    % Convierte los dos números escritos a valores de tipo float.
    values = sscanf(normalizedText, "%f %f");

    % Comprueba que se hayan ingresado exactamente dos números válidos.
    if numel(values) ~= 2 || any(~isfinite(values))
        fprintf("Entrada invalida. Ejemplo: 1.25,-0.50\n");
        continue;
    end

    % Extrae las coordenadas ingresadas.
    px = values(1);
    py = values(2);

    % Construye el mensaje con seis decimales y separado por coma.
    transmittedLine = sprintf("%.6f,%.6f", px, py);

    % Envía el mensaje al ESP32.
    % writeline agrega automáticamente el terminador configurado.
    writeline(esp32, transmittedLine);

    % Muestra en la consola el mensaje enviado.
    fprintf("MATLAB -> ESP32: %s\\r\\n\n", transmittedLine);
end

% Informa que el usuario terminó la consola.
fprintf("Consola finalizada.\n");

%% FUNCIÓN EJECUTADA AL RECIBIR UNA LÍNEA DEL ESP32

function showEsp32Line(source, ~)
try
    % Lee una línea completa recibida desde el ESP32.
    receivedLine = readline(source);

    % Muestra todos los mensajes recibidos, incluso si no son telemetría.
    fprintf("ESP32 -> MATLAB: %s\n", receivedLine);

    % Intenta interpretar el mensaje con el formato:
    % ARM,px,py,theta1_ref,theta2_ref,theta1,theta2
    values = sscanf(char(receivedLine), ...
                    "ARM,%f,%f,%f,%f,%f,%f");

    % Si el mensaje no contiene los seis valores esperados,
    % no actualiza las gráficas.
    if numel(values) ~= 6
        return;
    end

    % Extrae la posición cartesiana objetivo.
    px = values(1);
    py = values(2);

    % Extrae las referencias angulares calculadas por el ESP32.
    theta1Reference = values(3);
    theta2Reference = values(4);

    % Extrae los ángulos articulares medidos.
    theta1 = values(5);
    theta2 = values(6);

    % Recupera el estado y los objetos gráficos guardados.
    state = source.UserData;

    % Incrementa el contador de muestras recibidas.
    state.sample = state.sample + 1;

    % Añade los ángulos medidos y de referencia a las curvas.
    % Los valores recibidos en radianes se convierten a grados.
    addpoints(state.theta1MeasuredLine, state.sample, rad2deg(theta1));
    addpoints(state.theta1ReferenceLine, state.sample, rad2deg(theta1Reference));
    addpoints(state.theta2MeasuredLine, state.sample, rad2deg(theta2));
    addpoints(state.theta2ReferenceLine, state.sample, rad2deg(theta2Reference));

    % Longitudes de los dos eslabones del brazo, expresadas en centímetros.
    a1 = 8.3;
    a2 = 10.1;

    % Calcula la posición de la primera articulación mediante
    % la cinemática directa del primer eslabón.
    joint1X = a1 * cos(theta1);
    joint1Y = a1 * sin(theta1);

    % Calcula la posición del extremo del segundo eslabón.
    % El ángulo absoluto del segundo eslabón es theta1 + theta2.
    endX = joint1X + a2 * cos(theta1 + theta2);
    endY = joint1Y + a2 * sin(theta1 + theta2);

    % Actualiza la representación gráfica del brazo.
    % Los tres puntos son: base, articulación 1 y efector final.
    set(state.armLine, "XData", [0 joint1X endX], ...
                       "YData", [0 joint1Y endY]);

    % Actualiza la posición cartesiana objetivo.
    set(state.targetPoint, "XData", px, "YData", py);

    % Guarda nuevamente el contador y los objetos gráficos actualizados.
    source.UserData = state;

    % Actualiza la interfaz limitando la frecuencia de redibujado.
    % nocallbacks evita ejecutar callbacks gráficos durante la actualización.
    drawnow limitrate nocallbacks;

catch exception
    % Muestra cualquier error ocurrido durante la lectura o actualización.
    fprintf(2, "Error leyendo el ESP32: %s\n", exception.message);
end
end

%% FUNCIÓN DE CIERRE DEL PUERTO

function closeEsp32Port(port)
% Desactiva el callback de recepción serial.
configureCallback(port, "off");

% Descarta los datos pendientes en los búferes del puerto.
flush(port);
end