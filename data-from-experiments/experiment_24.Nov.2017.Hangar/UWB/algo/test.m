%%     
%     pause_time = 0.3*[time_diff; 2];
%     for j = 3:size(X,2) %1:size(X,2)-9
%         h2 = plot(X(1,j-2:j), X(2,j-2:j), '-ob'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
%         h3 = plot(real_X(1,j-2:j), real_X(2,j-2:j), '-ok');
%         str_title = sprintf('experiment%d; factorQ: %d; factorR: %d; j: %d', experimentNumber, factor_Q, factor_R, j);
%         title(str_title);
%         % Fram(j-4) = getframe(gcf);
%         pause(pause_time(j));
%         delete(h2);
%         delete(h3);
%     end
%% mitigation weighted linear combiation
%% measurement update
R = R_all;
Z_e = Z_e_all;
H_symbolic = H_all_symbolic;
% remove the measurement data with NaN
z = z_all(:, i); % measurement data
k = length(z);
index_ian = find(~isnan(z)); % is a number
while k >= 1
    if isnan(z(k)) % if no measurements are coming
        z(k) =[];
        Z_e(k) = [];
        H_symbolic(k,:) = [];
        R(k,:) = []; R(:,k) = [];
    end
    k = k-1;
end

z_length = length(z);
if z_length < 2
    continue
elseif z_length <= 3
        meas_trust_factor = 1;
        %             time_last = time_now;
        H = vpa(eval(subs( subs(H_symbolic, x_m, x_minus), N_x_n, positionOfNodes))); %<<<change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        R = R * R_affected_by_dt; % adjust R by dt
        R = R * meas_trust_factor; % adjust R by steps of ignored update due to missing measurements
        
        K_k = P_minus * H' / (H * P_minus * H' + R); %<<< R >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        %<<< add Z_e >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        residual = z - eval(subs( subs(Z_e, x_m, x_minus), N_x_n, positionOfNodes)); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        x_estimated = x_minus + K_k * (residual); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        X(:, i) = x_estimated;
        
        % refill the array with NaN in the index marked position,
        % exactly refill the NaN matrix with data
        for ijj = 1:length(index_ian) % is a number
            RESIDUAL(index_ian(ijj), end) = residual(ijj);
        end
        P(:, :, i) = vpa((eye(length(x_0)) - K_k * H) * P_minus);
        %K(:, :, i) = K_k;
        numCombinations= 1;
else    % z_length = 4 or 5
    residual_sum_array = [];
    x_estimated_array  = [];
    P_array = [];
    createNaNnum = z_length - 3;
    for nn = 0 : createNaNnum
        idxNaN = 	combnk(1:z_length, nn); % size #combinations * nn
        numCombinations = size(idxNaN,1) ;
        for idxNaN_i = 1:numCombinations % #combinations
            z(idxNaN_i) = nan; % elements with idx all turn into NaN
            k = length(z);
            while k >= 1
                if isnan(z(k)) % if no measurements are coming
                    z(k) =[];
                    Z_e(k) = [];
                    H_symbolic(k,:) = [];
                    R(k,:) = []; R(:,k) = [];
                end
                k = k-1;
            end
            
            H = vpa(eval(subs( subs(H_symbolic, x_m, x_minus), N_x_n, positionOfNodes))); %<<<change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   
            
            R_ = R * R_affected_by_dt; % adjust R by dt
            R_ = R_ * meas_trust_factor; % adjust R by steps of ignored update due to missing measurements
            
            K_k = P_minus * H' / (H * P_minus * H' + R_); %<<< R_ >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            %<<< add Z_e >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            residual = z - eval(subs( subs(Z_e, x_m, x_minus), N_x_n, positionOfNodes)); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            residual_sum = sum(residual)/length(residual);
            residual_sum_array = [residual_sum_array, residual_sum];
            x_estimated = x_minus + K_k * (residual); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            P = vpa((eye(length(x_0)) - K_k * H) * P_minus);
            x_estimated_array = [x_estimated_array, x_estimated];
            P_array = [P_array, P];
        end
                    
            %K(:, :, i) = K_k;
            inv_residual_sum_array = 1./(residual_sum_array);
            X(:, i) = sum(x_estimated_array .* inv_residual_sum_array) / sum(inv_residual_sum_array)  ;
            P(:, :, i)  = sum(P(:, :, i)  .* inv_residual_sum_array) / sum(inv_residual_sum_array)  ;
    end
end    

%{
                for nn = 0 : createNaNnum
                    idxNaN = 	combnk(1:z_length, nn); % size #combinations * nn
                    numCombinations = size(idxNaN,1) ;
                    P_array = zeros(4,4,numCombinations); % initial P_array
                    for idxNaN_i = 1:numCombinations % #combinations
                        z = z_orig;
                        if numCombinations > 1
                            z(idxNaN(idxNaN_i)) = nan; % elements with idx all turn into NaN
                            k = length(z);
                            while k >= 1
                                if isnan(z(k)) % if no measurements are coming
                                    z(k) =[];
                                    Z_e(k) = [];
                                    H_symbolic(k,:) = [];
                                    R(k,:) = []; R(:,k) = [];
                                end
                                k = k-1;
                            end
                        end
                        H = vpa(eval(subs( subs(H_symbolic, x_m, x_minus), N_x_n, positionOfNodes))); %<<<change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        
                        R_ = R * R_affected_by_dt; % adjust R by dt
                        R_ = R_ * meas_trust_factor; % adjust R by steps of ignored update due to missing measurements
                        
                        K_k = P_minus * H' / (H * P_minus * H' + R_); %<<< R_ >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        %<<< add Z_e >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        residual = z - eval(subs( subs(Z_e, x_m, x_minus), N_x_n, positionOfNodes)); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                        residual_sum = sum(residual)/length(residual);
                        residual_sum_array = [residual_sum_array, residual_sum];
                        x_estimated = x_minus + K_k * (residual); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                        P = vpa((eye(length(x_0)) - K_k * H) * P_minus);
                        x_estimated_array = [x_estimated_array, x_estimated];
                        P_array(:,:,idxNaN_i) = P;
                    end
                    
                    %K(:, :, i) = K_k;
                    inv_residual_sum_array = 1./(residual_sum_array);
                    % X(:, i) = sum(x_estimated_array .* inv_residual_sum_array) / sum(inv_residual_sum_array)  ;
                    X(:, i) = x_estimated_array * inv_residual_sum_array' / sum(inv_residual_sum_array)  ;
                    % P(:, :, i)  = sum(P_array  .* inv_residual_sum_array) / sum(inv_residual_sum_array)  ;
                    P(:, :, i)  = P_array * inv_residual_sum_array' / sum(inv_residual_sum_array)  ;
                end
}%