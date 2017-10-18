file_name = 'mat_positions6';
data_TXYt = importdata([file_name, '.mat']);

figure; hold on;
title(file_name);
plot(data_TXYt(:,2), data_TXYt(:,3), '+')
    pause(1.5);
for j = 10:size(data_TXYt,1)
    h2 = plot(data_TXYt(j-9:j,2), data_TXYt(j-9:j,3), '-or');
    daspect([10,10,10]);
    pause(0.03);
    delete(h2);
end