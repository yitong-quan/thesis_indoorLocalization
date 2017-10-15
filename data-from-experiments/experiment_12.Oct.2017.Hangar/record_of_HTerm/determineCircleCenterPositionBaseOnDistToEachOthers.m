
%% version2 ndoes oder x2, x3, x1, x5, x6
%     locations fixed: x1(0,0)  x3(+5.12,0)
P0 = rand(2,1)*10;
nodePositions = importdata('nodePos_by_determineNodesPositionBaseOnDistToEachOthers.mat');
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
[P,resnorm] = lsqnonlin(@myfun2,P0,[],[],options);

%axis square;
plot(nodePositions(1,:), nodePositions(2,:), '-d');
hold on;
plot(P(1,:), P(2,:), '-*');
daspect([10,10,10]);

function F = myfun2(P_center)
nodePositions = importdata('nodePos_by_determineNodesPositionBaseOnDistToEachOthers.mat');
dist2eachOther = zeros(5,1);
for i = 1:size(nodePositions,2)
        dist2eachOther(i) =  norm(nodePositions(:,i) - P_center) ;
end
% [(|0x2-center|),(|0x3-center|),(|0x1-center|),(|0x5-center|),(|0x6-center|)]
true_dist2eachOther = [4.20, 5.90, 4.93, 4.58, 5.33]';
F = dist2eachOther - true_dist2eachOther;
end