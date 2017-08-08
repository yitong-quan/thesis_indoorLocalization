clear 

%% 
% two D
%% flags
add_perturbance = 1; % 1 for add_perturbance, 0 for random seeding <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Flag
reasonable_gama = 1; % 1 for reasonable_gama, 0 for random gamma <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Flag
use_Gauss_Newton = 1;  % 1 for use_Gauss_Newton, 0 for not use_Gauss_Newton <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Flag
%%
%node_posi_sym = sym('n_%d', [1 2]);
node_posi_sym = sym('n_%d', [2 2]); % fix 1 out of the 2 ndoes
node_posi_sym(:,2) = [3, 0];
nodes_p = [-7, 3; 0, 0];
tag_p_sym = sym('tag_p_%d', [2 17]);
tag_p_x = linspace(-16,0,17);
tag_p = [tag_p_x; zeros(size(tag_p_x))]; 


f_sym = sym('f_%d', [length(node_posi_sym), length(tag_p_sym)]);
for j = 1:length(node_posi_sym)
    for i = 1:length(tag_p_sym)
        true_dist(j,i) = norm(tag_p(:,i) - nodes_p(:,j));
    end
end
true_dist = rand(size(true_dist)) + true_dist; % <<<<<<<<<<<<<<< <<<<<<<<<<<<<<<   add noise here, become noisy measurements
for j = 1:length(node_posi_sym)
    for i = 1:length(tag_p_sym)
        f_sym(j,i) = norm(tag_p_sym(:,i) - node_posi_sym(:,j)) - true_dist(j,i);
    end
end    
b = [];
for l = 1:size(f_sym)
    b = [b f_sym(l,:)];
end    
b = b.';
u = [tag_p_sym, node_posi_sym(:,1:end-1)].';
u = [u(:,1); u(:,2)]; % [tag_x,...,nodes_x,...,tag_y,...,nodes_y]
Q_sym = jacobian(b, u);
d_w_sym = Q_sym.' * b;
% cost function
F_sym = sum(sum((f_sym).^2));

%u_0 = [-6.3 -5.9 -4 -3 -2.0 -1 0, -7 3]'; %[ tag node]
%u_0 = [-7.3 -1.9 -9.9 -2.5 -2.9 -10.9 0.9, -7.9]'; %[ tag node] fix the last node

epslon_taget = 0.00001;
if add_perturbance == 1
    if use_Gauss_Newton == 1
        iter_max = 100;
    else
        iter_max = 300;
    end
else
    iter_max = 30;
end
u_0 = rand((length(node_posi_sym)+length(tag_p_sym)-1)*2,1);%[ tag node] fix the last node
u = u_0;
U = u_0;
u_tilde = inf;
gama_init = 0.3; %<<<<<0.5 also works
iter = 0;
cost = inf;
d_w_stack = [];
F = inf;
%cost = inf;
progress = @(trial, iter,u,cost) fprintf('trial %d, iter = %3d: u = %-32s, cost = %f\n', ...
    trial, iter, mat2str(u,6), cost);

COST_MIN = [];
cost_min = inf;
if add_perturbance == 1
    trialNum = 1;
else
    trialNum = 3;
end
for ii = 1:trialNum
    if add_perturbance == 0
        u_0 = rand((length(node_posi_sym)+length(tag_p_sym)-1)*2,1); %[ tag node] fix the last node
        u = u_0;
        u_tilde = inf;
        gama_init = 0.3; %<<<<<0.5 also works
        iter = 0;
        cost = inf;  
    end
    Gauss_Newton_Flag = 0; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Gauss_Newton_Flag
    while iter < iter_max && cost > epslon_taget %norm(u_tilde) >= epslon_taget
        d_w =  eval(subs(d_w_sym, [tag_p_sym(1,:), node_posi_sym(1,1:end-1), tag_p_sym(2,:), node_posi_sym(2,1:end-1)].', u));
        d_w_stack = [d_w_stack d_w];
        cost = eval(subs(F_sym, [tag_p_sym(1,:), node_posi_sym(1,1:end-1), tag_p_sym(2,:), node_posi_sym(2,1:end-1)].', u));
        progress(ii, iter, u, cost);
        F = [F,  cost];
        
        if norm(d_w) < 0.1
            if use_Gauss_Newton == 1
                Gauss_Newton_Flag = 1; %<<<<<<<<<<<<<<<<<<<<<<<<<<<<< set Flag
            end
            Q = eval(subs(Q_sym, [tag_p_sym(1,:), node_posi_sym(1,1:end-1), tag_p_sym(2,:), node_posi_sym(2,1:end-1)].', u));
        end
        
        if iter == 0
            u_tilde = gama_init * d_w;
        else
            if reasonable_gama == 0
                gama = rand*gama_init;  % set rand factor for gamma
            else
                u_last = U(:,end-1);
                denominator = norm(d_w - d_w_last)^2;
                if denominator ~= 0
                    if Gauss_Newton_Flag == 0 
                        gama = (u - u_last).' * (d_w - d_w_last) / denominator;
                    else 
                        gama = inv(Q'*Q); % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TODO badly scale
                    end
                    u_tilde = gama * d_w;
                else
                    if add_perturbance == 1  %%add_perturbance_at_local_minina_and_reached_global_minima
                        u_tilde = 10*rand * ones(size(d_w));
                        Gauss_Newton_Flag = 0 ;
                        if cost < cost_min
                            cost_min = cost;
                            u_most_likely = U(:,end-1);
                        end
                    else 
                        break;  %%random seeding
                    end                   
                end
            end
        end
        u = u - u_tilde;
        U = [U, u];
        iter = iter + 1;
        d_w_last = d_w;
    end
    if cost < cost_min
        cost_min = cost;
        u_most_likely = U(:,end-1);
    end
    COST_MIN = [COST_MIN, cost_min];
end

plot(F, '-*r');
u'
u_most_likely'
cost_min

