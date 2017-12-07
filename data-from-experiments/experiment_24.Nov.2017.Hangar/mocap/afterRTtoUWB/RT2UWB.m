x = importdata('..\..\alighment\RT_map_MoCap_to_UWB_theataT1T2.mat');
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
PoMC = cortexjson7sq(:,[4,5])/1000;
PoMC_RT =  M(:,[1,2]) * PoMC' + M(:,end);
figure;plot(PoMC_RT(1,:),PoMC_RT(2,:))
cortexjson7sq = [cortexjson7sq, PoMC_RT'];