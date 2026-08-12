%% Consola USB para cinematica inversa/directa y telemetria del ESP32
% CI px py           : coordenadas en centimetros
% CD theta1 theta2   : angulos en grados

%% CONFIGURACIÓN DE LA COMUNICACIÓN SERIAL

% Nombre del puerto serial donde se encuentra conectado el ESP32.
portName = "COM3";

% Velocidad configurada para la comunicación serial.
baudRate = 115200;

% Muestra en la consola el puerto al cual se intentará conectar.
fprintf("Conectando al ESP32 en %s...\n", portName);

%% APERTURA DEL PUERTO SERIAL

try
    % Crea el objeto de comunicación serial.
    % Timeout indica que MATLAB esperará como máximo un segundo
    % durante una operación de lectura.
    esp32 = serialport(portName, baudRate, "Timeout", 1);
catch exception
    % Si el puerto no puede abrirse, detiene la ejecución y muestra
    % el mensaje original de la excepción.
    error("No se pudo abrir %s. Cierre otros monitores seriales.\n%s", ...
        portName, exception.message);
end

% Crea un objeto de limpieza automática.
% Cuando el script termine, se ejecutará closeEsp32Port.
cleanupObject = onCleanup(@() closeEsp32Port(esp32));

% Configura la combinación CR/LF como terminador de los mensajes.
configureTerminator(esp32, "CR/LF");

%% CREACIÓN DE LA VENTANA GRÁFICA

% Crea la ventana que contendrá las gráficas de ángulos y posición.
figureHandle = figure("Name", "Brazo 2R - ESP32", ...
    "NumberTitle", "off");

%% GRÁFICA DE LOS ÁNGULOS ARTICULARES

% Crea el primer gráfico en la parte superior de la ventana.
angleAxes = subplot(2, 1, 1, "Parent", figureHandle);

% Permite colocar varias curvas sobre el mismo sistema de ejes.
hold(angleAxes, "on");

% Crea la línea animada del ángulo medido theta1.
theta1MeasuredLine = animatedline(angleAxes, "Color", [0 0.4470 0.7410], ...
    "LineWidth", 1.5, "DisplayName", "theta1");

% Crea la línea animada de la referencia de theta1.
% La línea discontinua permite diferenciarla del valor medido.
theta1ReferenceLine = animatedline(angleAxes, "Color", [0 0.4470 0.7410], ...
    "LineStyle", "--", "DisplayName", "theta1 ref");

% Crea la línea animada del ángulo medido theta2.
theta2MeasuredLine = animatedline(angleAxes, "Color", [0.8500 0.3250 0.0980], ...
    "LineWidth", 1.5, "DisplayName", "theta2");

% Crea la línea animada de la referencia de theta2.
theta2ReferenceLine = animatedline(angleAxes, "Color", [0.8500 0.3250 0.0980], ...
    "LineStyle", "--", "DisplayName", "theta2 ref");

% Activa la cuadrícula de la gráfica de ángulos.
grid(angleAxes, "on");

% Define las etiquetas de los ejes.
xlabel(angleAxes, "Muestra");
ylabel(angleAxes, "Angulo (rad)");

% Define el título de la gráfica.
title(angleAxes, "Seguimiento articular");

% Muestra la leyenda y selecciona automáticamente su mejor ubicación.
legend(angleAxes, "Location", "best");

%% GRÁFICA DE LA POSICIÓN CARTESIANA

% Crea el segundo gráfico en la parte inferior de la ventana.
armAxes = subplot(2, 1, 2, "Parent", figureHandle);

% Permite colocar varios elementos en el mismo sistema de ejes.
hold(armAxes, "on");

% Crea una línea animada para representar el recorrido real
% del efector final en el plano XY.
trajectoryLine = animatedline(armAxes, "Color", [0 0.4470 0.7410], ...
    "LineWidth", 1.2, ...
    "DisplayName", "Trayectoria medida");

% Crea un punto azul para representar la posición medida actual.
measuredPoint = plot(armAxes, 0, 0, "bo", "MarkerSize", 8, ...
    "MarkerFaceColor", "b", ...
    "DisplayName", "Posicion medida");

% Crea una cruz roja para representar la posición cartesiana objetivo.
targetPoint = plot(armAxes, 0, 0, "rx", "MarkerSize", 12, ...
    "LineWidth", 2, "DisplayName", "Objetivo");

% Activa la cuadrícula de la gráfica cartesiana.
grid(armAxes, "on");

% Usa la misma escala para ambos ejes.
axis(armAxes, "equal");

% Define los límites visibles del espacio de trabajo.
xlim(armAxes, [-20 20]);
ylim(armAxes, [-20 20]);

% Define las etiquetas de los ejes en centímetros.
xlabel(armAxes, "px (cm)");
ylabel(armAxes, "py (cm)");

% Define el título inicial de la gráfica cartesiana.
title(armAxes, "Posicion XY");

% Muestra la leyenda de la trayectoria, la posición y el objetivo.
legend(armAxes, "Location", "best");

%% ALMACENAMIENTO DEL ESTADO DE LA INTERFAZ

% Guarda el contador de muestras y los objetos gráficos dentro
% de UserData para que estén disponibles en showEsp32Line.
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

%% CONFIGURACIÓN DE LA RECEPCIÓN SERIAL

% Ejecuta showEsp32Line cada vez que MATLAB recibe una línea completa,
% detectada mediante el terminador CR/LF.
configureCallback(esp32, "terminator", @showEsp32Line);

%% INSTRUCCIONES MOSTRADAS AL USUARIO

% Informa que la conexión fue realizada correctamente.
fprintf("Conectado. Se mostraran logs y telemetria del ESP32.\n");

% Explica el comando de cinemática inversa.
fprintf("CI px py         -> cinematica inversa en centimetros.\n");

% Explica el comando de cinemática directa.
fprintf("CD theta1 theta2 -> cinematica directa en grados.\n");

% Muestra ejemplos de los dos tipos de comandos.
fprintf("Ejemplos: 'CI 13 13' o 'CD 30 15'.\n");

% Indica cómo terminar el programa.
fprintf("Escriba 'q' para cerrar.\n\n");

%% BUCLE PRINCIPAL DE LA CONSOLA

while true
    % Solicita al usuario un comando y elimina los espacios
    % ubicados al comienzo y al final.
    commandText = strtrim(input("comando > ", "s"));

    % Si el usuario escribe q, termina el bucle principal.
    if strcmpi(commandText, "q")
        break;
    end

    % Reemplaza las comas por espacios para permitir comandos escritos
    % con cualquiera de los dos separadores.
    normalizedText = strrep(commandText, ",", " ");

    % Divide el comando en elementos separados:
    % modo, primer valor y segundo valor.
    tokens = split(string(strtrim(normalizedText)));

    % Elimina elementos vacíos generados por espacios adicionales.
    tokens(tokens == "") = [];

    % Verifica que existan exactamente tres elementos.
    if numel(tokens) ~= 3
        fprintf("Entrada invalida. Use 'CI px py' o 'CD theta1 theta2'.\n");
        continue;
    end

    % Obtiene el modo de operación y lo convierte a mayúsculas.
    mode = upper(tokens(1));

    % Convierte el segundo y tercer elemento a valores numéricos.
    values = str2double(tokens(2:3));

    % Comprueba que el modo sea CI o CD y que los dos valores
    % ingresados sean números finitos.
    if (mode ~= "CI" && mode ~= "CD") || any(~isfinite(values))
        fprintf("Entrada invalida. Use 'CI px py' o 'CD theta1 theta2'.\n");
        continue;
    end

    % Para cinemática directa, comprueba que ambos ángulos se
    % encuentren dentro del intervalo de -90 a 90 grados.
    if mode == "CD" && any(abs(values) > 90)
        fprintf("Los angulos de CD deben estar entre -90 y 90 grados.\n");
        continue;
    end

    % Construye el mensaje que será enviado al ESP32.
    % El mensaje contiene el modo y dos valores con seis decimales.
    transmittedLine = sprintf("%s %.6f %.6f", ...
                              char(mode), values(1), values(2));

    % Envía el comando al ESP32 y añade automáticamente el
    % terminador CR/LF configurado anteriormente.
    writeline(esp32, transmittedLine);

    % Muestra en la consola el comando transmitido.
    fprintf("MATLAB -> ESP32: %s\\r\\n\n", transmittedLine);
end

% Informa que el bucle principal y la consola han finalizado.
fprintf("Consola finalizada.\n");

%% FUNCIÓN PARA PROCESAR LOS MENSAJES DEL ESP32

function showEsp32Line(source, ~)
try
    % Lee una línea completa recibida desde el ESP32.
    receivedLine = readline(source);

    % Muestra todos los mensajes recibidos, aunque no correspondan
    % al formato de telemetría.
    fprintf("ESP32 -> MATLAB: %s\n", receivedLine);

    % Divide el mensaje recibido usando la coma como separador.
    fields = split(strtrim(string(receivedLine)), ",");

    % Comprueba que el mensaje tenga diez campos y que empiece
    % con la palabra ARM. Si no cumple el formato, no se procesa.
    if numel(fields) ~= 10 || fields(1) ~= "ARM"
        return;
    end

    % Extrae el modo de funcionamiento reportado por el ESP32.
    mode = fields(2);

    % Convierte los ocho campos numéricos restantes a números.
    values = str2double(fields(3:10));

    % Si alguno de los datos no es un número válido, descarta el mensaje.
    if any(~isfinite(values))
        return;
    end

    % Coordenadas cartesianas de referencia.
    pxReference = values(1);
    pyReference = values(2);

    % Coordenadas cartesianas medidas o calculadas a partir
    % de los ángulos reales.
    pxMeasured = values(3);
    pyMeasured = values(4);

    % Referencias angulares de las dos articulaciones.
    theta1Reference = values(5);
    theta2Reference = values(6);

    % Ángulos medidos de las dos articulaciones.
    theta1 = values(7);
    theta2 = values(8);

    % Recupera el estado y los objetos gráficos almacenados.
    state = source.UserData;

    % Incrementa el contador de muestras recibidas.
    state.sample = state.sample + 1;

    % Agrega el ángulo medido theta1 a su curva.
    addpoints(state.theta1MeasuredLine, state.sample, theta1);

    % Agrega la referencia de theta1 a su curva.
    addpoints(state.theta1ReferenceLine, state.sample, theta1Reference);

    % Agrega el ángulo medido theta2 a su curva.
    addpoints(state.theta2MeasuredLine, state.sample, theta2);

    % Agrega la referencia de theta2 a su curva.
    addpoints(state.theta2ReferenceLine, state.sample, theta2Reference);

    % Añade la posición cartesiana medida a la trayectoria recorrida.
    addpoints(state.trajectoryLine, pxMeasured, pyMeasured);

    % Actualiza el punto que representa la posición medida actual.
    set(state.measuredPoint, "XData", pxMeasured, "YData", pyMeasured);

    % Actualiza el punto que representa la posición de referencia.
    set(state.targetPoint, "XData", pxReference, "YData", pyReference);

    % Actualiza el título de la gráfica indicando el modo reportado.
    title(state.armAxes, "Posicion XY - modo " + mode);

    % Guarda nuevamente el estado actualizado en el objeto serial.
    source.UserData = state;

    % Actualiza las gráficas limitando la frecuencia de redibujado.
    % nocallbacks evita ejecutar otros callbacks gráficos durante
    % esta actualización.
    drawnow limitrate nocallbacks;

catch exception
    % Muestra cualquier error producido durante la recepción,
    % interpretación o representación de la telemetría.
    fprintf(2, "Error leyendo el ESP32: %s\n", exception.message);
end
end

%% FUNCIÓN DE CIERRE DEL PUERTO SERIAL

function closeEsp32Port(port)
% Desactiva el callback encargado de recibir líneas.
configureCallback(port, "off");

% Vacía los datos pendientes en los búferes del puerto serial.
flush(port);
end