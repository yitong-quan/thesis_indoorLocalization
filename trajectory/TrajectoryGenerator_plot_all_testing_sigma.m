clear all; close all;
Xini = [0,0,1.4,1.4]';
% X = Xini; pdata(:,1) = Xini;
dt = 2/3; %% 0.3;
sampleNum = 1000;
A = [1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
G = [dt^2/2*eye(2);dt*eye(2)];
sigma = 0.1;
TrialNum = 0;
%w = randn(2,1,sampleNum);
NumberOfTraj = 50;
for j = 1:NumberOfTraj
    X = Xini;
    pdata(:,1,j) = Xini;
    TrialNum = TrialNum + 1;
    for i = 2:sampleNum
        % X = A*X + G*0.5*w(:,:,i);
        X = A * X + G * randn(2,1) * sigma;
%         if ( X(1) > 500 || X(1) < -500 ...
%                 || X(2) > 500 || X(2) < -500)
%             break
%         end
        pdata(:,i,j) = X;
%         if i == sampleNum
%             %subplot(5, 6, j);
%             figure(j)
%             plot(pdata(1,:,j),pdata(2,:,j), '*');
%             title(['j = ', num2str(j),'; sampleNum = ', num2str(sampleNum)]);
%         end
    end
    %figure(j)
     if max(pdata(1,:,j)) < 2000 & min(pdata(1,:,j)) > -2000
         if max(pdata(2,:,j)) < 2000 & min(pdata(2,:,j)) > -2000
             subplot(5, 10, j);
%             figure(j)
             plot(pdata(1,:,j),pdata(2,:,j));
             title(['j: ', num2str(j),'; #sample: ', num2str(sampleNum)])
         end
     end
end
% save('posivelodata2.mat', 'pdata');
