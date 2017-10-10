clear;
time_position = importdata('..\uwb03-Unnamed__Time_Position.mat');
figure; plot(time_position(:,2), time_position(:,3), '.');
i = length(time_position);
t_p_copy = [time_position zeros(length(time_position),1)];
% find the first non 0 position elements
for jj = 1:length(t_p_copy)
    if ~(t_p_copy(jj,2) == 0 && t_p_copy(jj,3) == 0 && t_p_copy(jj,4) == 0)
        last_t_p = t_p_copy(jj,:);
        disp(jj);
        break;
    end    
end 
for j = jj+1:length(t_p_copy)
    if ~(t_p_copy(j,2) == 0 && t_p_copy(j,3) == 0 && t_p_copy(j,4) == 0)
        disp(j);
        now_t_p = t_p_copy(j,:);
        diff_t_p = now_t_p - last_t_p;
        last_t_p = now_t_p;
        %velocities
        t_p_copy(j,2:4) = diff_t_p(2:4) / diff_t_p(1) /1000; %units: m/s
        t_p_copy(j, 5) = sqrt( sum( t_p_copy(j,2:4).^2 ) );
    end    
end 
%nodes 1 2 3 4. time: 1~6s, 18~21s(or 26~29s), 42~56s, 70~78s, 91~99s
nodes1= average4NodePosition(1, 6, time_position);
nodes2= average4NodePosition(18, 29, time_position);
nodes3= average4NodePosition(42, 56, time_position);
nodes4= average4NodePosition(70, 78, time_position);
nodes5= average4NodePosition(91, 99, time_position);
nodes_positions = [nodes1; nodes2; nodes3; nodes4; nodes5];
save('nodes_positions.mat','nodes_positions');

%{
while i>0
    if t_p_copy(i,2) == 0 && t_p_copy(i,3) == 0 && t_p_copy(i,4) == 0
        t_p_copy(i,:) = [];
    end
    i = i -1;
end    

diff_t_p = diff(t_p_copy);
velocity = diff_t_p(:,2:4) ./ diff_t_p(:,1);
%}