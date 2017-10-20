clear;
%%
%folder_name = 'positionBeforeMakedBetterFromJoan';
folder_name = 'positionsBetterFromJoan';
file_name = 'mat_positions';
experinumber = 6;
data_TXYt = importdata([folder_name, '/', file_name, num2str(experinumber), '.mat']);
time_diff = diff(data_TXYt(:,4));

h=figure; hold on;
title(['experiment', num2str(experinumber)]);
plot(data_TXYt(:,2), data_TXYt(:,3), '+');
    daspect([10,10,10]);
    
switch folder_name
    case 'positionsBetterFromJoan'
        savefig(h, [folder_name, '/USS experiment After', num2str(experinumber), '.fig']);
    case 'positionBeforeMakedBetterFromJoan'
        savefig(h, [folder_name, '/USS experiment before', num2str(experinumber), '.fig']);
    otherwise
        warning('please specify folder name: before or after making better')
end
       % pause(1.5);
pause_time = 0.00002*[time_diff; 2];        
for j = 10:size(data_TXYt,1)
    h2 = plot(data_TXYt(j-9:j,2), data_TXYt(j-9:j,3), '-or');
    Fram(j-9) = getframe(gcf);
    % pause(pause_time(j));
    delete(h2);
end

    video_str = ['videoUUS/video_USS_experiment', num2str(experinumber), '.avi'];
    video = VideoWriter(video_str);
    open(video)
    writeVideo(video, Fram)
    close(video)