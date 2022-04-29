load("scheduled_gains_PID.mat");
load("vs50_contol_system_data_base_plots\M_alpha_beta_deg.mat");

Ma = M_alpha_beta_deg(:,1);
Mb = M_alpha_beta_deg(:,2);

KI = -scheduled_gains_PID(:,1);
KP = -scheduled_gains_PID(:,2);
KD = -scheduled_gains_PID(:,3);

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

figure(1);
hold on;
for i=100:100:8200
    
    Plant = -Mb(i) / (s^2 - Ma(i));
    PID = pid( KP(i), KI(i), KD(i) );
    
    OL(i/100) = PID * Plant;
    CL(i/100) = feedback(PID*Plant,-1); % OL(i/100) / ( 1 + OL(i/100) );
    
%     bode(CL(i/100),options);
    drawnow();
end

%%

figure(2)
hold on;
for i=1:21
    bode(-CL(i),options);
%     step(-CL(i));
    drawnow();
    title('From t = 1s to t = 21s');
end
%%
figure(3)
hold on;
for i=22:38
    bode(-CL(i),options);
%     step(-CL(i));
    drawnow();
    title('From t = 21s to t = 38s');
end
%%
figure(4)
hold on;
for i=39:71
    bode(-CL(i),options);
%     step(-CL(i));
    drawnow();
    title('From t = 38s to t = 71s');
end
%%
figure(5)
hold on;
for i=72:82
    bode(-CL(i),options);
%     step(-CL(i));
    drawnow();
    title('From t = 71s to t = 82s');
end