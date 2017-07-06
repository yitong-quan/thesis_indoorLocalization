%% 
% %for i = [0.1 0.3 0.05 0.08 0.12 0.16]
% for i = 1:30
%     [X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(1, 0.5, 0, 0, num2str(i));
%     %[X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(1, 0.1, 0, 0, num2str(i));
%     %[X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(1, 0.1, 1, 100, num2str(i));
% end

%%
n= 0;
i_max = 1/9.1; 
j_max = 1/9.1;
i = 0.1/9;  % i should be 1
while i <= i_max % factor_Q
% while i <=0.1/3 % factor_Q    
    j = 0.1/9;
    while j <= j_max % factor_R
    %while j <= 0.1/3 % factor_R
%         KF_traj_noisy_meas_3nodes(i,j);
%         KF_traj_noisy_meas_4nodes(i,j);
        [X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(i, j, 1, 100, '29');
        fprintf('i:%f;      j:%f, j_max:%d;\n', i, j, j_max);
        j = j * 3;
        n=n+1;
    end
     i = i * 3;
end    