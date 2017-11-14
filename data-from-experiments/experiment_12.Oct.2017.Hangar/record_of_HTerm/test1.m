
for j = 5:size(X,2) %1:size(X,2)-9
    h2 = plot(X(1,j-4:j), X(2,j-4:j), '-+r'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
    str_title = sprintf('experiment%d; factorQ: %d; factorR: %d; j: %d', experimentNumber, factor_Q, factor_R, j);
    title(str_title);
    Fram(j-4) = getframe(gcf);
    pause(pause_time(j));
    delete(h2);
end
video_str = ['outliar_removed_video_ekf_experiment', num2str(experimentNumber), '11.14.avi'];
video = VideoWriter(video_str);
open(video)
writeVideo(video, Fram)
close(video)