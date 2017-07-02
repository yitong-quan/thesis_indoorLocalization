%%
%  figure
%  hold on
%  h1 = plot(1:10,'r');
%  h2 = plot(2:11,'g');
% 
%    %  set(h2,'Visible','off')


%% test for data reading after loop to generate traj

allData_copy = allData;
for i = 1:size(allData_copy,1)/4
    set0 = allData_copy(4*i-3:4*i,:);
    set = set0(:,2:end);
    disp(i-1)
    figure(); plot(set(1,:),set(2,:), 'b+');
    j = j+1;
end

%% sdfsasdasdada
