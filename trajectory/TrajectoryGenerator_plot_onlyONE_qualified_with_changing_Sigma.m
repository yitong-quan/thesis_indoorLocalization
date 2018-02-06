clear all; close all;
Xini = [0,0,0.8,0.8]'; %[0,0,1.0,1.0]'; %[0,0,1.4,1.4]';
% X = Xini; pdata(:,1) = Xini;
%dt = 0.025; % (2/3) /4; %% 0.3;
sampleNum = 3000; %30;
%A = [1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
% G = [dt^2/2*eye(2);dt*eye(2)];
sigma = 0.5; % 3.5;
TrialNum = 0;
LastTrialNum = 0;
%w = randn(2,1,sampleNum);
NumberOfTraj = 600;
allData = zeros(4, sampleNum + 1);
GoodTrajFound = 0;
while GoodTrajFound == 0
    fprintf('sigma=%f \n ', sigma);
    
    for j = 1:NumberOfTraj
        if GoodTrajFound == 1   
            break
        end    
        X = Xini;
        pdata(:,1) = Xini;
        TrialNum = TrialNum + 1;
        timestamp = [0];
        for i = 2:sampleNum
            % X = A*X + G*0.5*w(:,:,i);
%             randomNum = randn(2,1);
%             X = A * X + G * sigma * randomNum;
            if mod(i,5)==0
                dt = 0.5; % (2/3) /4; %% 0.3;
            else
                dt = 0.025; % (2/3) /4; %% 0.3;
            end
            timestamp = [timestamp,timestamp(end)+dt];
            A = [1,0,dt,0;0,1,0,dt;0,0,1,0;0,0,0,1];
            G = [dt^2/2*eye(2);dt*eye(2)];
            X = A * X + G * sigma * randn(2,1);
            % if ( X(1) > 10 || X(1) < -10 || X(2) > 10 || X(2) < -10)
            if ( X(1) > 7 || X(1) < -7 || X(2) > 7 || X(2) < -7)
                break
            end
            if  X(1) > 2.5
                X(3)=X(3) - 0.1*(X(1) - 2.5);
            end
            if  X(1) < -2.5
                X(3)=X(3) - 0.1*(X(1) + 2.5);
            end
            if  X(2) > 2.5
                X(4)=X(4) - 0.1*(X(2) - 2.5);
            end
            if  X(2) < -2.5
                X(4)=X(4) - 0.1*(X(2) + 2.5);
            end
            pdata(:,i) = X;
            if i == sampleNum
                %subplot(5, 6, j);
                figure()
                plot(pdata(1,:),pdata(2,:), 'r-+');
                title(['sigma:', num2str(sigma), '; j: ', num2str(j),'; sampleNum: ', num2str(sampleNum)]);
                GoodTrajFound = 1;
                allData = [allData ;[ [sigma, j, TrialNum - LastTrialNum, 0]', pdata ] ];
                %sigma = sigma - 0.1;
                LastTrialNum = TrialNum;
            end
        %break
        end
    end
    %sigma = sigma + 0.1;
end    
data =allData(5:8,:);
data = data(:,2:end);
data_t_dist = [timestamp;data];
% save('posivelodata2.mat', 'pdata');