
    
    pause_time = 0.3*[time_diff; 2];
    for j = 3:size(X,2) %1:size(X,2)-9 
        h2 = plot(X(1,j-2:j), X(2,j-2:j), '-ob'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r'); 
        h3 = plot(real_X(1,j-2:j), real_X(2,j-2:j), '-ok');
        str_title = sprintf('experiment%d; factorQ: %d; factorR: %d; j: %d', experimentNumber, factor_Q, factor_R, j);
        title(str_title);
        % Fram(j-4) = getframe(gcf);
        pause(pause_time(j));
        delete(h2);
        delete(h3);
    end