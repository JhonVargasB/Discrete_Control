close all
clear
clc

Ts = 0.02
a = 0.7068;
b = 3.0825;
G = [1  Ts ;
     0  a]
H = [0 ; b]
C = [1 0]
D = 0

t =0:Ts:10;
Nm = length(t)

Ga = [G zeros(2,1);-C*G 1]
Ha = [H ; -C*H]
Hb = [0 0 1]'
Ca = [C 0]
Da = 0

%% LQR
 Q = diag([1, 1, 120]);
R = 5;         % penalización del control

% Ganancia óptima LQR
Ka = dlqr(Ga, Ha, Q, R)

K = Ka(1,1:2)
Ki = -Ka(3)

Gac = Ga-Ha*Ka
Hac = Hb
Cac = Ca-Da*Ka
Dac = Da

 % Condiciones iniciales
 x0=[0; 0; 0]
 % Vector de entrada
 r = 0.3*ones(1,length(t));
 %sin(5 * t)
 %dlsim con CI y referencia

 [Yli,Xlia]=dlsim(Gac,Hac,Cac,Dac,r,x0);
 Xli = Xlia(:,1:2);
 Vli = Xlia(:,3);
 
 figure(1)
 stairs(t,Xli)
 hold on
 grid on
 legend('x1(k)','x2(k)','x3(k)','x4(k)')
 title('Estados con Integrador en atraso')
 
 figure(2)
 stairs(t,Yli)
 hold on
 stairs(t,r)
 legend('y','r')
 grid on
 title('Salida con Integrador en atraso')
 
 figure(3)
 stairs(t,Vli)
 hold on
 grid on
 title('v con integrador en atraso')
 
 u = -Ka*Xlia';
 figure(4)
 stairs(t,u)
 hold on
 grid on
 title('u con integrador en atraso')
