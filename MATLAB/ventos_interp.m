load('VENTO-1.DAT')
tempo = [0:0.01:85]';
vento_1(:,1) = interp1(VENTO_1(:,1),VENTO_1(:,2),tempo,'spline');
vento_1(:,2) = interp1(VENTO_1(:,1),VENTO_1(:,3),tempo,'spline');
vento_1(:,3) = zeros(8501,1);


csvwrite('vento_1.csv', vento_1)
