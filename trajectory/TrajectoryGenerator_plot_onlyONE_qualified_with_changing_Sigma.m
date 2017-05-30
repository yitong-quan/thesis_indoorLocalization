clear all; close all;
Xini = [0,0,1.0,1.0]'; %[0,0,1.4,1.4]';
% X = Xini; pdata(:,1) = Xini;
dt = 2/3; %% 0.3;
sampleNum = 1500;
A = [1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
G = [dt^2/2*eye(2);dt*eye(2)];
sigma = 1.0;
TrialNum = 0;
LastTrialNum = 0;
%w = randn(2,1,sampleNum);
NumberOfTraj = 1000000;
allData = zeros(4, sampleNum + 1);
while sigma > 0.01
    GoodTrajFound = 0;
    for j = 1:NumberOfTraj
        if GoodTrajFound == 1
            break
        end    
        X = Xini;
        pdata(:,1) = Xini;
        TrialNum = TrialNum + 1;
        for i = 2:sampleNum
            % X = A*X + G*0.5*w(:,:,i);
            X = A * X + G * sigma * randn(2,1);
            if ( X(1) > 100 || X(1) < -100 ...
                    || X(2) > 100 || X(2) < -100)
                break
            end
            pdata(:,i) = X;
            if i == sampleNum
                %subplot(5, 6, j);
                figure()
                plot(pdata(1,:),pdata(2,:), 'y+');
                title(['sigma:', num2str(sigma), '; j: ', num2str(j),'; sampleNum: ', num2str(sampleNum)]);
                GoodTrajFound = 1;
                allData = [allData ;[ [sigma, j, TrialNum - LastTrialNum, 0]', pdata ] ];
                %sigma = sigma - 0.1;
                LastTrialNum = TrialNum;
            end
        %break
        end
    end
    sigma = sigma - 0.02;
end    
% save('posivelodata2.mat', 'pdata');