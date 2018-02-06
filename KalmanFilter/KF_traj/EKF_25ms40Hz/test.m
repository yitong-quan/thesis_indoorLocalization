pos_diff_real_normal = [diff(real_X_normal(1,:));diff(real_X_normal(2,:))];
vel_x_real_normal = pos_diff_real_normal(1,:)./time_diff_normal';
vel_y_real_normal = pos_diff_real_normal(2,:)./time_diff_normal';

mis_vel_x_calib = X_normal(3,1:end-1)-vel_x_calib;
mis_vel_y_calib  = X_normal(4,1:end-1)-vel_y_calib;
mis_vel_sum_calib = sqrt(mis_vel_x_calib.^2 + mis_vel_y_calib.^2);


%% plot hist mis
% pos_diff_real_separated = [diff(real_X_separated(1,:));diff(real_X_separated(2,:))];
% vel_x_real_separated = pos_diff_real_separated(1,:)./time_diff_separated';
% vel_y_real_separated = pos_diff_real_separated(2,:)./time_diff_separated';
% 
% mis_vel_x_separated_form = X_separated(3,1:end-1)-vel_x_real_separated;
% mis_vel_y_separated_form = X_separated(4,1:end-1)-vel_y_real_separated;
% mis_vel_sum_separated_form = sqrt(mis_vel_x_separated_form.^2 + mis_vel_y_separated_form.^2);
% figure;histfit(X_separated(3,1:end-1)-vel_x_real_separated);
% figure;histfit(mis_vel_y_separated_form);
% figure; histfit(mis_vel_sum_separated_form,25,'rayleigh')
% set(findall(gcf,'-property','FontSize'),'FontSize',35)
% %savefig('mis_pos_y_calib');


%% plot velocity
% pos_diff_real_normal = [diff(real_X_normal(1,:));diff(real_X_normal(2,:))];
% vel_x_real_normal = pos_diff_real_normal(1,:)./time_diff_normal';
% vel_y_real_normal = pos_diff_real_normal(2,:)./time_diff_normal';
% 
% pos_diff_real_separated = [diff(real_X_separated(1,:));diff(real_X_separated(2,:))];
% vel_x_real_separated = pos_diff_real_separated(1,:)./time_diff_separated';
% vel_y_real_separated = pos_diff_real_separated(2,:)./time_diff_separated';
% 
% opt_tag_filled = [opt_tag_after_RRT(:,1:2),opt_tag_after_RRT(:,2),opt_tag_after_RRT(:,3:33),opt_tag_after_RRT(:,33),...
%     opt_tag_after_RRT(:,34:54),opt_tag_after_RRT(:,54),opt_tag_after_RRT(:,55:59),opt_tag_after_RRT(:,59),...
%     opt_tag_after_RRT(:,60:end)];
% vel_x_calib = diff(opt_tag_filled(1,:))./time_diff_normal';
% vel_y_calib = diff(opt_tag_filled(2,:))./time_diff_normal';
% 
% figure; plot(vel_x_real_normal,'r'); hold on; plot(X_normal(3,1:end-1),'k');
% plot(vel_x_calib,'b');plot([0.2:0.2:86-1],X_separated(3,1:end-5),'g');
% 
% figure; plot(vel_y_real_normal,'r'); hold on; plot(X_normal(4,1:end-1),'k');
% plot(vel_y_calib,'b');plot([0.2:0.2:86-1],X_separated(4,1:end-5),'g');


%% plot histfit of erorrs
% hh=figure;
% histfit(mis_pos(1,:));
% set(findall(gcf,'-property','FontSize'),'FontSize',35)
% savefig('mis_pos_x_calib');
% saveas(gcf,'mis_pos_x_calib.pdf');
% 
% figure;
% histfit(mis_pos(2,:));
% set(findall(gcf,'-property','FontSize'),'FontSize',35)
% savefig('mis_pos_y_calib');
% saveas(gcf,'mis_pos_y_calib.pdf');
% 
% figure;
% histfit(mis_dist,17,'rayleigh')
% set(findall(gcf,'-property','FontSize'),'FontSize',35)
% savefig('mis_dist_calib');
% saveas(gcf,'mis_dist_calib.pdf');
% 
%        hhh = figure;
% histfit(mis_v_x);
% set(findall(gcf,'-property','FontSize'),'FontSize',35)  
% savefig('mis_vel_x_calib');
% saveas(gcf,'mis_vel_x_calib.pdf');
% 
%     figure;
% histfit(mis_v_y);
% set(findall(gcf,'-property','FontSize'),'FontSize',35)
% savefig('mis_vel_y_calib');
% saveas(gcf,'mis_vel_y_calib.pdf');
% 
% figure;
% histfit(mis_v_sum,12,'rayleigh')
% set(findall(gcf,'-property','FontSize'),'FontSize',35)
% savefig('mis_vel_sum_calib');
% saveas(gcf,'mis_vel_sum_calib.pdf');