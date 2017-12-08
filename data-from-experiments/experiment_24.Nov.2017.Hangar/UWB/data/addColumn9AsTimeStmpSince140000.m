%%  time stamp starts at 15:00:00 , unit s
startPoint_TimeStamp = 1774.38600000000;
startPoint = data_t_dist(1,1);
timeStamp = data_t_dist(:,1) - startPoint + startPoint_TimeStamp;
figure; plot(data_t_dist(:,1), timeStamp)
daspect([10,10,10]);

% cortexjson6sq(1,9) = 1395.49500000000;
% startPoint = cortexjson6sq(1,2);
% cortexjson6sq(2:end,9) = cortexjson6sq(2:end,2) - startPoint + cortexjson6sq(1,9);
% figure; plot(cortexjson6sq(:,2), cortexjson6sq(:,9))

% startPoint_TimeStamp = 1786.14300000000;
% cortexjson7sq(516,9) = startPoint_TimeStamp;
% startPoint = cortexjson7sq(516,2);
% cortexjson7sq(517:end,9) = cortexjson7sq(517:end,2) - startPoint +startPoint_TimeStamp;
% cortexjson7sq(1:515,9) = cortexjson7sq(1:515,2) - startPoint +startPoint_TimeStamp;
% figure; plot(cortexjson7sq(:,2), cortexjson7sq(:,9))