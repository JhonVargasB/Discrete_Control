%% Controlador con integrador en adelanto
Ts = 0.02
a = 0.7068
b = 3.0825

t = 0:Ts:20;
G = [1  Ts ;
     0  a]
H = [0 ; b]
C = [1 0]
D = 0

Ga = [G [0;0];-C*G 1]
Ha = [H;-C*H]
Ca = [C 0]
Da = D
Hb = [0;0;1]

Ma = [Ha Ga*Ha Ga^2*Ha]
d = det(Ma)
r = rank(Ma)

Paz = poly(Ga)
a1 = Paz(2);a2 = Paz(3);a3 = Paz(4);
Wa = [   a2  a1   1
         a1   1   0
         1    0   0 ]
Ta = Ma*Wa    

%Polos de lazo cerrado
 sa =[-1.4+2.4i, -1.4-2.4i, -2.644 ]
 za = exp(Ts*sa)
 Pcz= poly(za)
 a11 = Pcz(2);a22 = Pcz(3);a33 = Pcz(4);
 Ka = [a33-a3 a22-a2 a11-a1]*inv(Ta)
 ki = -Ka(3)
 
 Gac = Ga-Ha*Ka
 Hac = Hb
 Cac = Ca-Da*Ka
 Dac = 0
 
 x0=[0 ;0 ;0]
 r = ones(1,length(t));
 
 [Yli,Xlia]=dlsim(Gac,Hac,Cac,Dac,r,x0);
 Xli = Xlia(:,1:2);
 Vli = Xlia(:,3);
 
 figure(1)
 stairs(t,Xli)
 hold on 
 grid on
 legend('x1(k)','x2(k)','x3(k)','x4(k)')
 title ('Estados integrador en adelanto')
 
 figure(2)
 stairs(t,Yli)
 hold on 
 grid on
 title ('Salida integrador en adelanto')
 
 figure(3)
 stairs(t,Vli)
 hold on 
 grid on
 title ('v integrador en adelanto')
