load("scheduled_gains_IPD.mat");
load("vs50_contol_system_data_base_plots\M_alpha_beta_deg.mat");

Ma = M_alpha_beta_deg(100:100:8200,1);
Mb = M_alpha_beta_deg(100:100:8200,2);

% ROBUSTO
KI = -scheduled_gains_PID(100:100:8200,1);
KP = -scheduled_gains_PID(100:100:8200,2);
KD = -scheduled_gains_PID(100:100:8200,3);

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

% figure(1);
% hold on;
for i=1:82
    %ATT
    Plant = -Mb(i) / (s^2 - Ma(i)) * TVA * DMARS * LPF * LPF;
%     PID = pid( -KP(i), -KI(i), -KD(i) );
    PID2 = (KP(i) + KI(i)*(1/s) + KD(i)*s);
    
%     OL(i/100) = PID * Plant;
    OL2(i) = PID2 * Plant;
%     CL(i/100) = feedback(PID*Plant,1); % OL(i/100) / ( 1 + OL(i/100) );
    
    CL2 = OL2(i) / ( 1 + OL2(i) );
    [Gm(i),Pm(i),Wcg(i),Wcp(i)] = margin(CL2);
    Gm_dB(i) = 20*log10(Gm(i));

%     %GUI
%     GUI_PLANT = CL(i/100) * Mg(i,1) * (1/s^2);
%     GUI_PID = pid( -GUI_KP(i), -GUI_KI(i), -GUI_KD(i) );
%     
%     GUI_OL(i/100) = GUI_PID * GUI_PLANT;
%     GUI_CL(i/100) = feedback(GUI_PID*GUI_PLANT,1);
%     bode(CL(i/100),options);
%     drawnow();
    disp(i);
end

Robust_att_metrics = [Gm_dB,Gm,Pm,Wcg,Wcp];

%% OPEN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
hold on;

% Define the color order based on the number of models
col = parula(83); %
    
for i=1:21
    bode(OL2(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-3 1e2])
    drawnow();
    title('Comparison of Robust Attitute Open Loop Bode Plot - (1s - 21s)');
end
ylim([-900,270])
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
    bode(OL2(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-3 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Open Loop Bode Plot - (22s - 32s)');
end
ylim([-900,270]);
%%
figure(3)
hold on;
for i=33:39
    bode(OL2(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-3 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Open Loop Bode Plot - (33s - 39)');
end
ylim([-900,270]);
%%
figure(4)
hold on;
for i=40:71
    bode(OL2(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-3 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Open Loop Bode Plot - (40s - 71s)');
end
ylim([-900,270]);
%%
figure(5)
hold on;
for i=72:82
    bode(OL2(i),options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-3 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Open Loop Bode Plot - (72s - 82s)');
end
ylim([-900,270]);

%% CLOSED LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6)
hold on;

% Define the color order based on the number of models
col = parula(83); %
    
for i=1:17
    bode( OL2(i)/(1+OL2(i)), options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-4 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Closed Loop Bode Plot - (1s - 17s)');
end
ylim([-360,540]);
%%
cb = colorbar('Location','eastoutside')
cb.Label.String = 'Elevation (ft in 1000s)';
aux = (1/82) * [0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80];
colorbar('Ticks',aux, ...
     'TickLabels',{'0','5','10','15','20','25','30','35','40','45','50','55','60','65','70','75','80'});
% 'Label', 'Flight Time (s)');

%%
figure(7)
hold on;
  
for i=18:26
    bode( OL2(i)/(1+OL2(i)), options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-4 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Closed Loop Bode Plot - (18s - 26s)');
end
ylim([-360,540]);
%%
figure(8)
hold on;
for i=27:39
    bode( OL2(i)/(1+OL2(i)), options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-4 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Closed Loop Bode Plot - (27s - 39s)');
end
ylim([-360,540]);
%%
figure(9)
hold on;
for i=40:70
    bode( OL2(i)/(1+OL2(i)), options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-4 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Closed Loop Bode Plot - (40s - 70s)');
end
ylim([-360,540]);

%%
figure(10)
hold on;
for i=71:82
    bode( OL2(i)/(1+OL2(i)), options, 'b');
    % Find handles of all lines in the figure that have the color blue
    lineHandle = findobj(gcf,'Type','line','-and','Color','b');
    % Change the color to the one you defined
    set(lineHandle,'Color',col(i,:),'linewidth', 2);
%     step(-CL(i));
    xlim([1e-4 1e2]);
    drawnow();
    title('Comparison of Robust Attitute Closed Loop Bode Plot - (71s - 82s)');
end
ylim([-360,540]);