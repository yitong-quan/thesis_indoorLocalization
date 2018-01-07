%i:j = 0.0003 to 0.003 good, 0.001 best

j = 0.0001; %0.0001;

RMSD_array = [];
while j < 10
    i = 0.0001;
    while i < 10
      [~,~,~,RMSD_] = KF_using_HTerm_data(i,j,3);
      RMSD_array = [RMSD_array, [RMSD_;i;j]];
      i =i*2;
    end
    j = j*2;
end