
for i = [0.1 0.3 0.05 0.08 0.12 0.16]
    [X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(i, 10, 1, 100, '01');
    
end
