clear;
x1C1C = [
    300.4416961
    408.0646259
    517.4698113
    632.5764706
    717.8955224
    ];
x2020 = [
    323.3044983
    436.893039
    549.6566901
    653.9411765
    744.9866667
    ];
x3E3E = [
    287.3270677
    402.8181818
    501.7418738
    618.6266667
    716.5243902
    ];
x4D4D = [
    308.0534979
    421.4725458
    525.1101836
    649.4642857
    738.195122
    ];
x6E6E = [
    310.4501718
    418.5392491
    524.2990476
    638.2117647
    717.92
    ];
x1C1C = x1C1C(1:4,:);
x2020 = x2020(1:4,:);
x3E3E= x3E3E(1:4,:);
x4D4D = x4D4D(1:4,:);
x6E6E = x6E6E(1:4,:);
true_value = [300;400;500;600];
Tag_0x1C1C = (x1C1C + 5) / 100 / 0.9925;
Tag_0x2020 = (x2020 + 0.24217) / 100 / 0.9993;
Tag_0x3E3E = (x3E3E + 14) / 100 / 1.00547; 
Tag_0x4D4D = (x4D4D + 8) / 100 / 1.01452;
Tag_0x6E6E = (x6E6E + 0) / 100 / 0.9993;
