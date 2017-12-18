UWB_idx = [65,83,112,131,180];
MC_idx = [74,97,122,146,193];

UWB_ts = data(UWB_idx,1);
MC_ts = MoCap_data_shrinked(MC_idx,9);
time_gaps =UWB_ts- MC_ts;
time_gaps_mean = mean(time_gaps); % time_gaps_mean = -8.36253712625157
idx_alighment_UWB_MC = [UWB_idx; MC_idx];
save('..\..\alighment\idx_alighment_UWB_MC_exp3.mat', 'idx_alighment_UWB_MC');