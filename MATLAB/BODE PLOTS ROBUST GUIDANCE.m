load("scheduled_gains_IPD.mat");
load("vs50_contol_system_data_base_plots\M_alpha_beta_deg.mat");
load('scheduled_gui_IPD.mat');
load('M_gamma.mat');

Ma = M_alpha_beta_deg(100:100:8200,1);
Mb = M_alpha_beta_deg(100:100:8200,2);
Mg = M_gamma(100:100:8200,1);

% ROBUSTO
KI = -scheduled_gains_PID(100:100:8200,1);
KP = -scheduled_gains_PID(100:100:8200,2);
KD = -scheduled_gains_PID(100:100:8200,3);

GUI_KP = scheduled_gui_IPD(100:100:8200,2);
GUI_KI = scheduled_gui_IPD(100:100:8200,1);
GUI_KD = scheduled_gui_IPD(100:100:8200,3);


% JOSEF ETTL
% KP = PID_deg(:,1);
% KI = PID_deg(:,2);
% KD = PID_deg(:,3);

s = tf('s');


options = bodeoptions;
options.FreqUnits = 'Hz';
options.Grid = 'on';
options.Title.FontSize = 20;
options.TickLabel.FontSize = 20;
options.XLabel.FontSize = 20;
options.YLabel.FontSize = 20;

TVA = exp(-0.011*s) / ( 1+2*0.697*0.013*s+0.013^2*s^2 );
DMARS = 1 / ( 1 + 2*0.707*0.01*s + 0.01^2*s^2 );
LPF = 1 / ( 1 + 2*0.5*(1/40)*s + (1/40)^2*s^2 );

Gm  = zeros(82,1);
Pm  = zeros(82,1);
Wcg = zeros(82,1);
Wcp = zeros(82,1);
Gm_dB  = zeros(82,1);

%%
% figure(1)
% figure(2)
% hold on;

% Define the color order based on the number of models
col = parula(83); %
for i=5:15
    %ATT
    ATT_Plant = -Mb(i) / (s^2 - Ma(i)) * TVA * DMARS * LPF * LPF;
%     PID = pid( -KP(i), -KI(i), -KD(i) );
    ATT_PID2 = (KP(i) + KI(i)*(1/s) + KD(i)*s);
    
%     OL(i/100) = PID * Plant;
    ATT_OL2(i) = ATT_PID2 * ATT_Plant;
    
    ATT_CL = ATT_OL2(i) / ( 1 + ATT_OL2(i) );
    
    %GUI
    GUI_PLANT = ATT_CL * (Mg(i)/s^2);
%     GUI_PID = pid( -GUI_KP(i), -GUI_KI(i), -GUI_KD(i) );
    GUI_PID2 = (GUI_KP(i) + GUI_KI(i)*(1/s) + GUI_KD(i)*s);

% GUIDANCE CLOSED LOOP:
    GUI_OL = GUI_PID2 * GUI_PLANT;
%     GUI_CL(i) = feedback(GUI_PID*GUI_PLANT,1);
    
% GUIDANCE CLOSED LOOP:
    GUI_CL = GUI_OL / ( 1 + GUI_OL );
    [Gm(i),Pm(i),Wcg(i),Wcp(i)] = margin(GUI_CL);
    Gm_dB(i) = 20*log10(Gm(i));

% %     bode(GUI_OL, {1e-2,1e2}, options, 'b');
%     bode(GUI_CL,{1e-2,1e2}, options, 'b');
%     % Find handles of all lines in the figure that have the color blue
%     lineHandle = findobj(gcf,'Type','line','-and','Color','b');
%     % Change the color to the one you defined
%     set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     drawnow();
    disp(i);
end

for i=45:75
    %ATT
    ATT_Plant = -Mb(i) / (s^2 - Ma(i)) * TVA * DMARS * LPF * LPF;
%     PID = pid( -KP(i), -KI(i), -KD(i) );
    ATT_PID2 = (KP(i) + KI(i)*(1/s) + KD(i)*s);
    
%     OL(i/100) = PID * Plant;
    ATT_OL2(i) = ATT_PID2 * ATT_Plant;
    
    ATT_CL = ATT_OL2(i) / ( 1 + ATT_OL2(i) );
    
    %GUI
    GUI_PLANT = ATT_CL * (Mg(i)/s^2);
%     GUI_PID = pid( -GUI_KP(i), -GUI_KI(i), -GUI_KD(i) );
    GUI_PID2 = (GUI_KP(i) + GUI_KI(i)*(1/s) + GUI_KD(i)*s);

% GUIDANCE CLOSED LOOP:
    GUI_OL = GUI_PID2 * GUI_PLANT;
%     GUI_CL(i) = feedback(GUI_PID*GUI_PLANT,1);
    
% GUIDANCE CLOSED LOOP:
    GUI_CL = GUI_OL / ( 1 + GUI_OL );
    [Gm(i),Pm(i),Wcg(i),Wcp(i)] = margin(GUI_CL);
    Gm_dB(i) = 20*log10(Gm(i));

% %     bode(GUI_OL, {1e-2,1e2}, options, 'b');
%     bode(GUI_CL,{1e-2,1e2}, options, 'b');
%     % Find handles of all lines in the figure that have the color blue
%     lineHandle = findobj(gcf,'Type','line','-and','Color','b');
%     % Change the color to the one you defined
%     set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     drawnow();
    disp(i);
end
% % title({'Comparison of Robust Guidance Open Loop Bode Plot (5s - 15s)'});
% title({'Comparison of Robust Guidance Closed Loop Bode Plot (45s - 75s)'});
% xlim([1e-2 1e1])
% ylim([-180,360])

Robust_gui_metrics = [Gm_dB,Gm,Pm,Wcg,Wcp];

%%

figure(3);
% figure(4);
hold on;

for i=45:75
    %ATT
    ATT_Plant = -Mb(i) / (s^2 - Ma(i)) * TVA * DMARS * LPF * LPF;
%     PID = pid( -KP(i), -KI(i), -KD(i) );
    ATT_PID2 = (KP(i) + KI(i)*(1/s) + KD(i)*s);
    
%     OL(i/100) = PID * Plant;
    ATT_OL2(i) = ATT_PID2 * ATT_Plant;
    
    ATT_CL(i) = ATT_OL2(i) / ( 1 + ATT_OL2(i) );
    
    %GUI
    GUI_PLANT = ATT_CL(i) * (Mg(i)/s^2);
%     GUI_PID = pid( -GUI_KP(i), -GUI_KI(i), -GUI_KD(i) );
    GUI_PID2 = (GUI_KP(i) + GUI_KI(i)*(1/s) + GUI_KD(i)*s);

% GUIDANCE CLOSED LOOP:
    GUI_OL = GUI_PID2 * GUI_PLANT;
%     GUI_CL(i) = feedback(GUI_PID*GUI_PLANT,1);
    
% GUIDANCE CLOSED LOOP:
    GUI_CL = GUI_OL / ( 1 + GUI_OL );

%     bode(GUI_OL,{1e-2,1e2}, options, 'b');
    bode(GUI_CL,{1e-2,1e2}, options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
    drawnow();
end
% title({'Comparison of Robust Guidance Open Loop Bode Plot (45s - 75s)'});
title({'Comparison of Robust Guidance Closed Loop Bode Plot (45s - 75s)'});
xlim([1e-2 1e1])
ylim([-180,360])


%% OPEN LOOP

%%
for i=45:75
    bode(GUI_OL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title({'Comparison of Guidance Open Loop Bode Plot','(5s - 15s) and (45s - 75s)'});
end

%% CLOSED LOOP
figure()
hold on;

% Define the color order based on the number of models
col = parula(83); %
    
for i=5:15
    bode(GUI_CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
end
for i=45:75
    bode(GUI_CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title({'Comparison of Guidance Closed Loop Bode Plot','(5s - 15s) and (45s - 75s)'});
end


















%% OPEN LOOP
figure()
hold on;

% Define the color order based on the number of models
col = parula(83); %
    
for i=1:21
    bode(OL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('\bf Comparison of Guidance Open Loop Bode Plot - (1s - 21s)');
end
%%
cb = colorbar('Location','eastoutside')
cb.Label.String = 'Elevation (ft in 1000s)';
aux = (1/82) * [0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80];
colorbar('Ticks',aux, ...
     'TickLabels',{'0','5','10','15','20','25','30','35','40','45','50','55','60','65','70','75','80'});
% 'Label', 'Flight Time (s)');

%%
figure(2)
hold on;
  
for i=22:31
    bode(OL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Open Loop Bode Plot - (22s - 31s)');
end
%%
figure(3)
hold on;
for i=32:38
    bode(OL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Open Loop Bode Plot - (32s - 38s)');
end
%%
figure(4)
hold on;
for i=39:71
    bode(OL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Open Loop Bode Plot - (39s - 71s)');
end
%%
figure(5)
hold on;
for i=72:82
    bode(OL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Open Loop Bode Plot - (72s - 82s)');
end

%% CLOSED LOOP
figure()
hold on;

% Define the color order based on the number of models
col = parula(83); %
    
for i=1:21
    bode(CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Closed Loop Bode Plot - (1s - 21s)');
end
%%
cb = colorbar('Location','eastoutside')
cb.Label.String = 'Elevation (ft in 1000s)';
aux = (1/82) * [0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80];
colorbar('Ticks',aux, ...
     'TickLabels',{'0','5','10','15','20','25','30','35','40','45','50','55','60','65','70','75','80'});
% 'Label', 'Flight Time (s)');

%%
figure(2)
hold on;
  
for i=22:32
    bode(CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Closed Loop Bode Plot - (22s - 32s)');
end
%%
figure(3)
hold on;
for i=33:38
    bode(CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Closed Loop Bode Plot - (33s - 38s)');
end
%%
figure(4)
hold on;
for i=39:49
    bode(CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Closed Loop Bode Plot - (39s - 49s)');
end
%%
figure()
hold on;
for i=50:71
    bode(CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Closed Loop Bode Plot - (50s - 71s)');
end
%%
figure(5)
hold on;
for i=72:82
    bode(CL(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    drawnow();
    title('Comparison of Guidance Closed Loop Bode Plot - (72s - 82s)');
end