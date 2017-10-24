close
x_p = estimated_posi_with_timeSt(1,:);
y_p = estimated_posi_with_timeSt(2,:);
x_v = estimated_posi_with_timeSt(3,:);
y_v = estimated_posi_with_timeSt(4,:);
circle_center = P;


dist = sqrt((x_p-circle_center(1)).^2 + (y_p-circle_center(2)).^2);
n = 1:length(dist);
plot(dist, '-x');
hold on;
adj = 5;
sds = 2.3;
[y,i,xmedian,xsigma] = hampel(dist,adj,sds); %x_p(2,2), (y_p,3,0.9)
plot(n,[1;1]*xmedian+sds*[-1;1]*xsigma)
plot(find(i),dist(i),'sk')
% plot(y_p);
% plot(x_v);
% plot(y_v);
plot(y, '-o')
hold off;
figure; plot(y, '-o')