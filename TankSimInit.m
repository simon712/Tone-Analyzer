%(* ::Package:: *)

close all; clear all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TankSimInit.m
% Fil f\[ODoubleDot]r att simulera stegsvar f\[ODoubleDot]r PID reglering och kaskadraglering av
% dubbeltankprocessen.
% Uppdatera parametrarna med era framr\[ADoubleDot]knade v\[ADoubleDot]rden.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Definiera processens konstanter f\[ODoubleDot]r Tank 1
K1  = 3.6;
T1  = 16.1;
D1  = 0.6;

% Definiera \[ODoubleDot]verf\[ODoubleDot]ringsfunktion f\[ODoubleDot]r Tank 1
% Andra termen \[ADoubleDot]r en 2a ordningens Pade approximation av d\[ODoubleDot]tiden
[num den] = pade(D1,2);
G1 = tf([K1],[T1 1])*tf(num,den);

%%% Definiera konstanter f\[ODoubleDot]r Tank 2
K2  = 1;
T2  = T1; % samma som f\[ODoubleDot]r Tank 1
D2  = 0;  % kan f\[ODoubleDot]rsummas

% Definiera \[ODoubleDot]verf\[ODoubleDot]ringsfunktion f\[ODoubleDot]r Tank 1
G2 = tf([K2],[T2 1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%% Definiera PID-Regulatorns konstanter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1) F\[ODoubleDot]r fasmarginal 40 grader
K_pid_40 = 2.37;
T_i_40   = 21.79;
T_d_40   = 3.44;
T_f_40   = 2.37;
% Definiera \[ODoubleDot]verf\[ODoubleDot]ringsfunktionen f\[ODoubleDot]r PID regulatorn
F_pid_40 = K_pid_40*(1 + tf([1],[T_i_40 0]) + tf([T_d_40 0],[T_f_40 1]) );
                   
% 2) F\[ODoubleDot]r fasmarginal 60 grader
K_pid_60 = 1.92;
T_i_60   = 27.86;
T_d_60   = 6.02;
T_f_60   = 2.91;
% Definiera \[ODoubleDot]verf\[ODoubleDot]ringsfunktionen f\[ODoubleDot]r PID regulatorn
F_pid_60 = K_pid_60*(1 + tf([1],[T_i_60 0]) + tf([T_d_60 0],[T_f_60 1]) );
        
% Slutna systemets \[ODoubleDot]verf\[ODoubleDot]ring f\[ODoubleDot]r de b\[ADoubleDot]gge PID regulatorerna 
G_r2h2_PID_40 = F_pid_40*G1*G2/(1+F_pid_40*G1*G2);
G_r2h2_PID_60 = F_pid_60*G1*G2/(1+F_pid_60*G1*G2);

% Kolla om den framr\[ADoubleDot]knade fasenmarginalen st\[ADoubleDot]mmer
[Gm,Pm_40_PID,Wg,Wp] = margin(F_pid_40*G1*G2);
[Gm,Pm_60_PID,Wg,Wp] = margin(F_pid_60*G1*G2);
Pm_40_PID
Pm_60_PID

% Plotta stegsvar f\[ODoubleDot]r de b\[ADoubleDot]gga PID regulatorerna
figure(1)
subplot(1,2,1)
step(G_r2h2_PID_40)
hold on
step(G_r2h2_PID_60)
legend('Fasmarginal 40','Fasmarginal 60')
title('Stegsvar PID Reglering')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kaskadregelringen 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Konstanterna f\[ODoubleDot]r den inre loopens PI-regulator
Kp_1 = 2.778;
Ti_1 = T1;

% \[CapitalODoubleDot]verf\[ODoubleDot]ringsfunktionen f\[ODoubleDot]r PI-regulatorn till den inre loopen
Fpi_1 = Kp_1*(1 + tf([1],[Ti_1 0]));

% Krets\[ODoubleDot]verf\[ODoubleDot]ringen i den inre loopen
% Andra termen \[ADoubleDot]r en 2a ordningens Pade approximation av d\[ODoubleDot]tiden
L_1 = Kp_1*K1*tf([1],[Ti_1 0])*tf(num,den);

% Slutna \[ODoubleDot]verf\[ODoubleDot]ringsfunktionen f\[ODoubleDot]r den inre loopen 
G_r1h1 = L_1/(1 + L_1);

% Yttre loopens PI-Regulator, fasmarginal 40 grader
Kp_2_40 = 0.68;
Ti_2_40 = 3.786;

% \[CapitalODoubleDot]verf\[ODoubleDot]ringsfunktionen f\[ODoubleDot]r PI-regulatorn  i den yttre loopen - 40 graders fasmarginal
F_pi2_40 = Kp_2_40*(1 + tf([1],[Ti_2_40 0]));

% Yttre loopens PI-Regulator, fasmarginal 60 grader
Kp_2_60 = 1.253;
Ti_2_60 = 8.613;

% \[CapitalODoubleDot]verf\[ODoubleDot]ringsfunktionen f\[ODoubleDot]r PI-regulatorn  i den yttre loopen - 60 graders fasmarginal
F_pi2_60 = Kp_2_60*(1 + tf([1],[Ti_2_60 0]));

% Slutna systemets \[ODoubleDot]verf\[ODoubleDot]ring f\[ODoubleDot]r de b\[ADoubleDot]gge kaskadreglerade systemen
G_r2h2_kaskad_40 = F_pi2_40*G_r1h1*G2/(1+F_pi2_40*G_r1h1*G2);
G_r2h2_kaskad_60 = F_pi2_60*G_r1h1*G2/(1+F_pi2_60*G_r1h1*G2);

% Kolla om den framr\[ADoubleDot]knade fasenmarginalen st\[ADoubleDot]mmer
[Gm,Pm_40_Cascade,Wg,Wp] = margin(F_pi2_40*G_r1h1*G2);
[Gm,Pm_60_Cascade,Wg,Wp] = margin(F_pi2_60*G_r1h1*G2);
Pm_40_Cascade
Pm_60_Cascade


% Plotta stegsvar f\[ODoubleDot]r de b\[ADoubleDot]gge kaskadreglerade systemen
subplot(1,2,2)
step(G_r2h2_kaskad_40)
hold on
step(G_r2h2_kaskad_60)
legend('Fasmarginal 40','Fasmarginal 60')
title('Stegsvar Kaskadreglering')

% Plotta stegsvaren f\[ODoubleDot]r alla regulatorerna i samma plot
figure(2)
step(G_r2h2_PID_40)
hold on
step(G_r2h2_PID_60)
step(G_r2h2_kaskad_40)
step(G_r2h2_kaskad_60)
legend('PID - Fasmarginal 40','PID - Fasmarginal 60', 'Kaskad - Fasmarginal 40', 'Kaskad - Fasmarginal 60')
title('Stegsvar Kaskadreglering')
