x = importdata('..\..\alighment\RT_map_MoCap_to_UWB_theataT1T2.mat');
% x(1) = 3.11; x(2) =x(2)+0.35;
%x(1) = 3.11; x(2) =x(2);
% x(1) = 3.10;
%x(3) =x(3)-0.1;
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
PoMC = cortexjson7sq(:,[4,5])/1000;
PoMC_RT =  M(:,[1,2]) * PoMC' + M(:,end);
% figure;
plot(PoMC_RT(1,:),PoMC_RT(2,:),'b')
cortexjson7sq = [cortexjson7sq, PoMC_RT'];
hold on;
plot(X(1,:),X(2,:),'r');
hold off;