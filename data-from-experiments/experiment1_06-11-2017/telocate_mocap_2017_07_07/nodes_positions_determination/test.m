%% calculate distances matrix between nodes
nodes_positions = [4161.77182689655,2235.61214448276,1121.24218333333;184.822603869210,-143.127276884650,1208.33112492280;-1741.84732630000,2032.12649985000,1185.33600880000;-1396.30540772784,-2433.57009459426,1255.07934958802;2938.41844377029,-3013.26989169788,1196.59444252185];
nodes_p = nodes_positions(:,1:2);
nodes_p = nodes_p/1000; % units: mm>>>m
n1 = nodes_p(5,:);
n2 = nodes_p(2,:);
n3 = nodes_p(1,:);
n4 = nodes_p(4,:);
n5 = nodes_p(3,:);
n = [n1; n2; n3; n4; n5];
dist_betw_nodes = zeros(size(nodes_p,1));

for i = 1:5
    for j = 1:5
        diff_x = n(j,1) - n(i,1);
        diff_y = n(j,2) - n(i,2);
        dist_betw_nodes(i,j) = sqrt(diff_x^2 + diff_y^2);
    end
end    
d_min = 10000;

%% calculate possibilities of each node in each position
for i = 1:5
    for j = 1:5
        d = abs(dist_betw_nodes(i,j) - 5.458442728034734);
        if d < d_min
            disp(i); disp(j);
            d_min = d;
        end
    end
end  