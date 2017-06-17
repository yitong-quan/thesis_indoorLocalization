%% sdasdas
% % this function generate the Jacobian Matrix 'H_matrix' of the expected measuremnts 
% % matrix 'z_expected'(#nodes,1) to time_updatedstate vector 'x_minu'
% %
% % output 'H_matr'(#nodes,sieze(x_minu,1))
% % input state vector 'x_minu', positionOfNodes 'posNodes'(#nodes,2),
% 
% 
% function H_matr = H_martix_EKF_Symbolic(x_minu, posNodes, r,~,~)
%     % expected measuremnts matrix 'z_expected'(#nodes,1)
%     z_expected = sqrt(sum((posNodes(:,1) - x_minu).^2));
% end

%% positionOfNodes = [-50 -50; 100 -50; 100 100; -50 100]'; 
%% VERSION 1; WITH ALL INNER NOTATION
clear all: close all;
syms n_x1 n_x2 n_x3 n_y1 n_y2 n_y3 z_e1 z_e1 z_e1 x_m1 x_m2 x_m3 x_m4
numNodes = 3; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
% posNodes
% n_x = sym('n_x', [1 numNodes]);
% n_y = sym('n_y', [1 numNodes]);
% N_xy = [n_x; n_y]
N_x_n = [n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]

% time_updated state vector
%x_m = sym('x_m', [4 1])
x_m = [x_m1; x_m2; x_m3; x_m4]

% Z_e: expected measurements
%Z_e = sym('z_e', [numNodes 1])
% z_e1 = sqrt( (n_x1 - x_m1)^2 + (n_y1 - x_m2)^2 )
% z_e2 = sqrt( (n_x2 - x_m1)^2 + (n_y2 - x_m2)^2 )
% z_e3 = sqrt( (n_x3 - x_m1)^2 + (n_y3 - x_m2)^2 )
Z_e = [
       sqrt( (n_x1 - x_m1)^2 + (n_y1 - x_m2)^2 );
       sqrt( (n_x2 - x_m1)^2 + (n_y2 - x_m2)^2 );
       sqrt( (n_x3 - x_m1)^2 + (n_y3 - x_m2)^2 )
       ]
H1 = jacobian(Z_e, x_m)

%% VERSION 2; WITH OUTER MATRIX NOTATION
%clear all: close all;
syms N_xy x_m Z_e
numNodes = 3; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
% posNodes
% n_x = sym('n_x', [1 numNodes]);
% n_y = sym('n_y', [1 numNodes]);
% N_xy = [n_x; n_y]
% N_xy = sym('n%d%d', [2 numNodes]) %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]
N_x_n = sym('N_x_n%d%d', [2 numNodes]) %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the ist node 

% time_updated state vector
x_m = sym('x_m%d', [4 1])


% Z_e: expected measurements
%Z_e = sym('z_e', [numNodes 1])
% z_e1 = sqrt( (n_x1 - x_m1)^2 + (n_y1 - x_m2)^2 )
% z_e2 = sqrt( (n_x2 - x_m1)^2 + (n_y2 - x_m2)^2 )
% z_e3 = sqrt( (n_x3 - x_m1)^2 + (n_y3 - x_m2)^2 )
Z_e = [     % [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
       sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
       sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
       sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 )
       ]
H2 = jacobian(Z_e, x_m)
A = [5; 4; 0; 0]
B = [-50 -50;100 -50; 100 100]'
%H2_evulation = subs(H2, [x_m, N_x_n], [A, B]);
H2_evulation = eval(subs( subs(H2, x_m, A), N_x_n, B))
%eval(H2_evulation)





