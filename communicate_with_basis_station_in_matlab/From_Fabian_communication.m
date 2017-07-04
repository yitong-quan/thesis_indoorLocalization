delete(instrfindall)
s=serial('COM10','BaudRate',115200);
% s2=serial('COM43','BaudRate',115200);
fopen(s)
% fopen(s2)
A=[];
% A2=[];
t=[];
tic;
for n=1:10000
    % [A2(n,1),count] = fscanf(s2,'%d');
    [A(n,1),count] = fscanf(s,'%d');
    t(n,1)=toc;
    D=(A2-A);
    plot(t,D)
    drawnow
    pause(0.01);
end
fclose(s)
fclose(s2)
delete(instrfindall)