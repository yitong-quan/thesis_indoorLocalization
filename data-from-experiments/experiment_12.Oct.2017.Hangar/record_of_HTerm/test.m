%% plot traj moving
     h = plot(X(1,:), X(2,:), 'ob');
     hold on;
    X= estimated_posi_with_timeSt;
	pause_time = 0.2*[time_diff'; 2];
    for j = 51:size(X,2) %1:size(X,2)-9 
        h2 = plot(X(1,j-4:j), X(2,j-4:j), '-+r'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r'); 
            daspect([10,10,10]);
            title(num2str(j));
        % pause(pause_time(j));
        delete(h2);
        
    end