    
%%
% x_pred: prediction state vector
% 
syms x_pred A x_prior               Z_e N_xy

x_prior = sym('x_prior%d', [4 1]); % prior state vector
P_prior = sym('P_prior%d', [4 4]); % prior state covariance matrix %P_prior = 7*eye(4);
A = 2*ones(4);%A = sym('A%d', [4 4]);  % 
Q = eye(4);%Q = sym('Q%d', [4 4]);  % 
H = 3*ones(4);%%H = sym('H%d', [4 4]);  % 
R = 5*eye(4);%R = sym('R%d', [4 4]);  % 
z = sym('z%d', [4 1]);  % 

% time update
x_pred = A*x_prior
P_pred = simplify( A*P_prior*A' + Q )

% measurements update
K = simplify( P_pred * H' / (H * P_pred * H' + R))
x_estimated = simplify( x_pred + K * (z - H * x_pred))
P_estimated = simplify( (eye(4) - K * H) * P_pred)






% 
% 
%     numNodes = nodes_Nums; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%     N_x_n = sym('N_x_n%d%d', [2 numNodes]); % posNodes %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the ist node 
%     x_m = sym('x_m%d', [4 1]); % time_updated state vector