%%
openfig('output\3_RRT\3_after_RRT.fig');
ws_3 = importdata('output\3_RRT\3_ekf_last_3_loops_after_rrt_work_space.mat');
X = ws_3.X; % KF output trajectory (format: x-; y- position; x- ; y- velocities)
data = ws_3.data; % timestamps and distance matrix data from HTerm 
data = data(1:end-1,:);
timeStamp = ws_3.timeStamp;
time_diff = ws_3.time_diff; % time gap between each two adjacent sample s
j = 3; % number of adjacent points of trajectory shown in video
h2 = plot(X(1,j-2:j), X(2,j-2:j), '-ob'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
h3 = plot(X(1,j), X(2,j), '-*b');
            frm_num = 0;
            stepsize = 0.03;
    for tt = 0:stepsize:data(end,1) 
                    str_title = sprintf('j: %d',j);
            title(str_title);
        frm_num = frm_num + 1;
            Fram(frm_num) = getframe(gcf);
            %pause(pause_time(j));
        if abs(data(j,1) - tt) <= (stepsize/2-0.000000000001)
            j = j+1;
            children = get(gca, 'children');
            children(1).XData = X(1,j); % children(1) -- X(1,j), X(2,j)
            children(1).YData = X(2,j);
            children(2).XData = X(1,j-2:j); % children(2) -- X(1,j-2:j), X(2,j-2:j)
            children(2).YData = X(2,j-2:j);
            %children(3); % children(3) -- nodes
        end
    end
    video_str = ['video_ekf_experiment_3e-2.avi'];
    video = VideoWriter(video_str);
    
    open(video)
    writeVideo(video, Fram)
    close(video)
    
%     j = 9;
%     for tt = 0:0.001:data(end,1) 
%             h2 = plot(X(1,j-2:j), X(2,j-2:j), '-ob'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
%             h3 = plot(X(1,j), X(2,j), '-*b');
%             str_title = sprintf('j: %d',j);
%             title(str_title);
%             Fram(tt) = getframe(gcf);
%             %pause(pause_time(j));
%             delete(h2);
%             delete(h3);
%         if data(j,1) - tt <= 0.0007
%             j = j+1;
%         end
%     end
%     video_str = ['video_ekf_experiment', num2str(experimentNumber), '.avi'];
%     video = VideoWriter(video_str);
%     open(video)
%     writeVideo(video, Fram)
%     close(video)