%% calibration_from_HTerm_to_tag

% 0x1C1C   what is shown in HTerm is: 0.9925 * UWB_data[n] * 100 - 5.205
x1C1C = [236.3636364 491.787234042553 712.28 918.823529411765 1180.66666666667];
Tag_0x1C1C = (x1C1C + 5.205) / 100 / 0.9925;