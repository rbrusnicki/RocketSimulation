
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\V.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\q.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\Pdin.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\M_alpha_beta_deg.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\Ixx.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\Cnr.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\Cmq.mat");
% load("C:\Users\Brusnicki\Desktop\vs50_contol_system_data_base_plots\Clp.mat");

load("vs50_contol_system_data_base_plots\V.mat");
load("vs50_contol_system_data_base_plots\q.mat");
load("vs50_contol_system_data_base_plots\Pdin.mat");
load("vs50_contol_system_data_base_plots\M_alpha_beta_deg.mat");
load("vs50_contol_system_data_base_plots\Ixx.mat");
load("vs50_contol_system_data_base_plots\Cnr.mat");
load("vs50_contol_system_data_base_plots\Cmq.mat");
load("vs50_contol_system_data_base_plots\Clp.mat");

expo = [-2:0.001:2];
w = 10.^expo;
k = length(w);

%% Guidance control
s = tf('s');

% ATT_PT2 = 1 / (1+ 0.244*s + 0.0337*s^2);              % Ettl's version
ATT_PT2 = 1.4723^2 / (s^2 + 2*0.5*1.4723*s + 1.4723^2 ); % match -3dB at average of plots
% ATT_PT2_2 = 1 / ( 1 + 2*0.5*0.6792*s + (0.6792^2)*s^2);  %same as before
% ATT_PT2 = 1  / (1+ 0.244*(2*s) + 0.0337*(2*s)^2);      % Ettl's version doubling s
% ATT_PT2 = 1 *(s/5.6+1) / (1+ 0.244*(2*s) + 0.0337*(2*s)^2);      % Ettl's version doubling s adding zero
% ATT_PT2 = 1 *(1.5*s/5.6+1) / (1+ 0.244*(1.5*2*s) + 0.0337*(1.5*2*s)^2);      % Ettl's version doubling s - adding zero -  s times 1.5
% ATT_PT2 = (10/(s+10) )*(1.5*s/4.6+1) / (1+ 0.244*(1.5*2*s) + 0.0337*(1.5*2*s)^2);      % Ettl's version doubling s - adding zero -  s times 1.5- added pole


[magpt2,phasept2,~] = bode(ATT_PT2,w);

for j=1:k
   magPT2(1,j) = magpt2(1,1,j);
   phasePT2(1,j) = phasept2(1,1,j);
end
magPT2_dB = 10 * log10(magPT2);
 
%% NEED TO PLOT THIS FIRST -> COLORFUL BODEPLOTS - CLOSED LOOP 
subplot(2,1,1);
hold on;
plot(w, magPT2_dB, '--');

subplot(2,1,2);
hold on;
plot(w, phasePT2, '--');


%%
s = tf('s');
ATT_PT2 = 1 / (1+ 0.244*s + 0.0337*s^2);              % Ettl's version

% GuiP = (3.0) / acc(100*i,2);
% GuiI = (0.0) / acc(100*i,2);
% GuiD = (20*3)/ acc(100*i,2);
% 
% Gui_PID(i,1) = GuiP + GuiI * (1/s) + GuiD * s;

Gui_PID = ( 3 + 0 * (1/s) + 60 * s ) / (180/pi);
figure()
bode(Gui_PID*ATT_PT2*(1/s^2))
grid
set(findall(gcf,'type','line'),'linewidth',2)
title('Guidance Control - Open Loop Bode Plot')
 