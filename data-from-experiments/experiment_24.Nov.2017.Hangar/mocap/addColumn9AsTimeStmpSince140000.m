%%  time stamp starts at 14:00:00 , unit s
% circle(1,9) = 891.1810;
% startPoint = circle(1,2);
% circle(2:end,9) = circle(2:end,2) - startPoint + circle(1,9);
% 
% figure; plot(circle(:,2), circle(:,9))

% cortexjson6sq(1,9) = 1395.49500000000;
% startPoint = cortexjson6sq(1,2);
% cortexjson6sq(2:end,9) = cortexjson6sq(2:end,2) - startPoint + cortexjson6sq(1,9);
% 
% figure; plot(cortexjson6sq(:,2), cortexjson6sq(:,9))

startPoint_TimeStamp = 1786.14300000000;
cortexjson7sq(516,9) = startPoint_TimeStamp;
startPoint = cortexjson7sq(516,2);
cortexjson7sq(517:end,9) = cortexjson7sq(517:end,2) - startPoint +startPoint_TimeStamp;
cortexjson7sq(1:515,9) = cortexjson7sq(1:515,2) - startPoint +startPoint_TimeStamp;

figure; plot(cortexjson7sq(:,2), cortexjson7sq(:,9))