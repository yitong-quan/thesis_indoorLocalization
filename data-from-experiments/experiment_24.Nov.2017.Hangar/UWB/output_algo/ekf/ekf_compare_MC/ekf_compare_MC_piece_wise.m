% figure;
% Loop_Idx = [48,133;134,218;220,288];
% plot(X(1,(48:133)), X(2,(48:133)), 'b+-');
% hold on;
% plot(real_X(1,(48:133)), real_X(2,(48:133)), 'r+-');
%     title('1');
%     daspect([10,10,10]);
% figure;
% plot(X(1,(134:218)), X(2,(134:218)), 'b+-');
% hold on;
% plot(real_X(1,(134:218)), real_X(2,(134:218)), 'r+-');
%     title('2');
%     daspect([10,10,10]);
% figure;
% plot(X(1,(222:288)), X(2,(222:288)), 'b+-');
% hold on;
% plot(real_X(1,(222:288)), real_X(2,(222:288)), 'r+-');
%     title('3');
%     daspect([10,10,10]);

figure;
subplot(1,3,1)
Loop_Idx = [48,133;134,218;220,288];
plot(X(1,(48:133)), X(2,(48:133)), 'b+-');
hold on;
plot(real_X(1,(48:133)), real_X(2,(48:133)), 'r+-');
    title('1');
    daspect([10,10,10]);
subplot(1,3,2)
plot(X(1,(134:218)), X(2,(134:218)), 'b+-');
hold on;
plot(real_X(1,(134:218)), real_X(2,(134:218)), 'r+-');
    title('2');
    daspect([10,10,10]);
subplot(1,3,3)
plot(X(1,(222:288)), X(2,(222:288)), 'b+-');
hold on;
plot(real_X(1,(222:288)), real_X(2,(222:288)), 'r+-');
    title('3');
    daspect([10,10,10]);