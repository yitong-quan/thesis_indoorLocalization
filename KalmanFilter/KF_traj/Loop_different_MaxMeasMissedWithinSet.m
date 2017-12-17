
%for i = [0.1 0.3 0.05 0.08 0.12 0.16]
for i = [0.011111
        0.033333 
0.100000 
0.300000 
0.900000 
2.700000 
8.100000 
24.300000 
72.900000 
]'
    %[X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(i, 10, 1, 100, '01');
    [X, P, z_meas] = KF_traj_noisy_meas_missing_data_within_each_set(i, 1/90, 0, -100, '01');
end
