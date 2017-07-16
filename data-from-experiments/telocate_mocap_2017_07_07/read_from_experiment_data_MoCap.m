%% check for numbers among NaN for the last 3 column
format longG
%D = importdata('uwb02-Unnamed.trc');
j1 = 0;j2 = 0;j3 = 0;
for i = 1:length(OrigDataRate) % F: index. check if there is data recorded into the last 3 column, which should be NaN in normal case
    if  ~isnan(OrigDataRate(i))
        j1 = j1 + 1;
        F(j1,1) = i;
    end
    if  ~isnan(OrigDataStartFrame(i))
        j2 = j2 + 1;
        F(j2,2) = i;
    end
    if  ~isnan(OrigNumFrames(i))
        j3 = j3 + 1;
        F(j3,3) = i;
    end
end
%%check if each row elements in F is with the same value
if ~nnz(F(:,1) - F(:,2)) == 0 % nnz: Number of nonzero matrix element
    error('Error, last two columns dont have same index with number_data')
elseif ~nnz(F(:,1) - F(:,3)) == 0 % false when all zero
    error('Error, last first and third columns dont have same index with number_data')
end
%% replace data
%~isnan(OrigDataRate')

% X = NumFrames(6:end);
% Y = NumMarkers(6:end);
% Z = Units(6:end);
% posi = [X Y Z];
posi = [NumFrames NumMarkers Units];
% replace the ii row of posi with the INDEX row of the last 3 columns
% data.INDEX is the elements in variable F 
for ii = 1: size(F,1)
    ind = F(ii,1); 
    posi(ind,:) = [OrigDataRate(ind) OrigDataStartFrame(ind) OrigNumFrames(ind)];  
end
% TEMP = [OrigDataRate(1278:1291) OrigDataStartFrame(1278:1291) OrigNumFrames(1278:1291)];
% posi(1273:1286,:) = TEMP;
position = posi(6:end,:);
position(isnan(position)) = 0 ; % replace NaN with 0, due to data missing
time_stamp = CameraRate(6:end);
Time_Position = [time_stamp position];
%plot(position(:,1), position(:,2), '-*');
plot(Time_Position(:,2), Time_Position(:,3), '-*');
