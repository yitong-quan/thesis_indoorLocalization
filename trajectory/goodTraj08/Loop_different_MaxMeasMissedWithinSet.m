
%for i = [0.1 0.3 0.05 0.08 0.12 0.16]
for i = 1:30
    [X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(1, 0.5, 0, 0, num2str(i));
    %[X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(1, 0.1, 0, 0, num2str(i));
    %[X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(1, 0.1, 1, 100, num2str(i));
end
