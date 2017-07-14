%% calibration_from_HTerm_to_tag
format longG
% 0x1C1C   what is shown in HTerm is: 0.9925 * UWB_data[n] * 100 - 5.205
x1C1C = [236.3636364 491.787234042553 712.28 918.823529411765 1180.66666666667];
Tag_0x1C1C = (x1C1C + 5.205) / 100 / 0.9925; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% try Tag_0x1C1C = (x1C1C + 5) / 100 / 0.9925; since 0.205 is already wraped up in Hterm
HTermShouldBe = 0.9925 * Tag_0x1C1C * 100 - 5.205;
if all((HTermShouldBe - x1C1C) < 0.000001)
    disp('0x1C1C calibration good')
else
    disp('0x1C1C opps....!!!!!!!!!!!!!')
end

% 0x2020   what is shown in HTerm is: 0.9993 * UWB_data[n] * 100 - 0.24217
x2020 = [311.767441860465               349.2637363                     502.5                  587.3125];
Tag_0x2020 = (x2020 + 0.24217) / 100 / 0.9993; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% try Tag_0x2020 = (x2020 + 0.24217) / 100 / 0.9993; since 0.24217 is already wraped up in Hterm
HTermShouldBe = 0.9993 * Tag_0x2020 * 100 - 0.24217;
if all((HTermShouldBe - x2020) < 0.000001)
    disp('0x2020 calibration good')
else
    disp('0x2020 opps....!!!!!!!!!!!!!')
end

% 0x3E3E   what is shown in HTerm is: 1.00547 * UWB_data[n] * 100 - 14.103
x3E3E = [276.232558139534                         0          534.739130434783          683.708333333333];
Tag_0x3E3E = (x3E3E + 14.103) / 100 / 1.00547; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% try Tag_0x3E3E = (x3E3E + 14) / 100 / 1.00547; since 0.103 is already wraped up in Hterm
HTermShouldBe = 1.00547 * Tag_0x3E3E * 100 - 14.103;
if all((HTermShouldBe - x3E3E) < 0.000001)
    disp('0x3E3E calibration good')
else
    disp('0x3E3E opps....!!!!!!!!!!!!!')
end

% 0x4D4D   what is shown in HTerm is: 1.01452 * UWB_data[n] * 100 - 8.1253
x4D4D = [273.558139534883          317.622641509434           483.95652173913          661.583333333333];
Tag_0x4D4D = (x4D4D + 8.1253) / 100 / 1.01452; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% try Tag_0x4D4D = (x4D4D + 8) / 100 / 1.01452; since 0.01452 is already wraped up in Hterm
HTermShouldBe = 1.01452 * Tag_0x4D4D * 100 - 8.1253;
if all((HTermShouldBe - x4D4D) < 0.000001)
    disp('0x4D4D calibration good')
else
    disp('0x4D4D opps....!!!!!!!!!!!!!')
end

% 0x6E6E   what is shown in HTerm is: 0.9993 * UWB_data[n] * 100 - 0.24217
x6E6E = [289.976744186046                         0          471.239130434783                  596.5625];
Tag_0x6E6E = (x6E6E + 0.24217) / 100 / 0.9993; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
HTermShouldBe = 0.9993 * Tag_0x6E6E * 100 - 0.24217;
if all((HTermShouldBe - x6E6E) < 0.000001)
    disp('0x4D4D calibration good')
else
    disp('0x4D4D opps....!!!!!!!!!!!!!')
end