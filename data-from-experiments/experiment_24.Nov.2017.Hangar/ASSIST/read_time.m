for i = 1: size(fusiontest,1)
    timestamp=num2str(fusiontest(i,1));
    msec(i)=str2double(timestamp(11:13))/1000;
    timewhole(i,:) = datestr(fusiontest(i,1)/86400/1000 + datenum(1970,1,1));
    nametemp = strsplit(timewhole(i,:),{'-',' ',':'});
    timmimu(i) = str2double(nametemp(5));
    timsimu(i) = str2double(nametemp(6))+msec(i);
end
timeStamp_in_second = timmimu*60+timsimu;
timewhole
figure;plot(timeStamp_in_second);
figure;plot(fusiontest(:,1));