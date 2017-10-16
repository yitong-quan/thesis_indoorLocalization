file_name = 'mat_positions4';
data_TXY = importdata([file_name, '.mat']);

figure; hold on;
title(file_name);
plot(data_TXY(:,2), data_TXY(:,3), '+')
    pause(1.5);
for j = 10:size(data_TXY,1)
    h2 = plot(data_TXY(j-9:j,2), data_TXY(j-9:j,3), '-or');
    daspect([10,10,10]);
    pause(0.03);
    delete(h2);
end