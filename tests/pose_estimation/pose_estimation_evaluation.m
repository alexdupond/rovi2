close all;
clear;
rots = 8;
%%
files = dir('/home/mikkel/workspace/rovi2/rovi2/tests/pose_estimation/data_multi_objects/*.csv');
fullpaths = fullfile({files.folder}, {files.name});
n = length(fullpaths);
%real_is_valid_all = [];
for i=1:n  
[pathstr,name,ext] = fileparts(string(fullpaths(1,i)));
%%
data = readtable(string(fullpaths(1,i)),'Delimiter',',','ReadVariableNames', false);
is_valid = logical(data{31:32:end,2}');
est_valid_poses = sum(is_valid);
rotations = [linspace(0,-(rots/2-1)/(rots/2)*pi,rots/2) linspace(pi,1/(rots/2)*pi,rots/2)];
rot_z_gt = repelem(rotations,10);
rot_z_gt_valid = rot_z_gt(is_valid);
rot_z_gt_not_valid = rot_z_gt(~is_valid);
%% x- and y- axis rotation
rot_x = data{30:32:end,3}';
rot_y = data{30:32:end,4}';
%% z-axis rotation
rot_z = data{30:32:end,2}';
rot_z_error = rot_z-rot_z_gt;

rot_z_error_correction = (rot_z_error > pi)*-2*pi + (rot_z_error < -pi)*2*pi;
rot_z_error_corrected = rot_z_error + rot_z_error_correction;
rot_z_corrected = rot_z_gt+rot_z_error_corrected;
%%
rot_z_valid = rot_z_corrected(is_valid);
rot_z_not_valid = rot_z_corrected(~is_valid);

figure('Name',['z_rotation_' char(name)])
scatter(rot_z_gt_valid, rot_z_valid, 'MarkerEdgeColor', 'g')
hold on
scatter(rot_z_gt_not_valid, rot_z_not_valid, 'MarkerEdgeColor', 'r')
plot(rot_z_gt,rot_z_gt)
plot([-(rots/2-1)/(rots/2)*pi pi], [-(rots/2-1)/(rots/2)*pi+0.2 pi+0.2],'r:')
plot([-(rots/2-1)/(rots/2)*pi pi], [-(rots/2-1)/(rots/2)*pi-0.2 pi-0.2],'r:')
legend({'Estimated vs actual rotation (valid)', 'Estimated vs actual rotation (not valid)', 'Ground truth rotation', 'Threshold'}, 'Location', 'northwest')
xlabel('Actual rotation [rad]')
ylabel('Estimated rotation [rad]')
xlim([-(rots/2-1)/(rots/2)*pi-0.2 pi+0.2])
ylim([-2*pi 2*pi])
saveas(gcf,['z_rotation_' char(name)],'epsc')
%% xyz translation
xyz = [data{26:32:end,5},data{27:32:end,5},data{28:32:end,5}]';
xyz_median = median(xyz,2);
xyz_gt = xyz_median;
%xyz_gt = [-0.20; 0.22; 0];
xyz_error_euclidian = sqrt(sum((xyz-xyz_gt).^2, 1));
xyz_valid = xyz_error_euclidian(is_valid);
xyz_not_valid = xyz_error_euclidian(~is_valid);

figure('Name',['xyz_translation_' char(name)])
scatter(rot_z_gt_valid, xyz_valid, 'MarkerEdgeColor', 'g')
hold on
scatter(rot_z_gt_not_valid, xyz_not_valid, 'MarkerEdgeColor', 'r')
plot([-(rots/2-1)/(rots/2)*pi pi], [0.03 0.03], 'r:')
legend({'Estimated translation vs actual rotation (valid)', 'Estimated translation vs actual rotation (not valid)', 'Threshold'} , 'Location', 'northwest')
xlabel('Actual rotation [rad]')
ylabel('Distance from actual xyz-position [m]')
xlim([-(rots/2-1)/(rots/2)*pi-0.2 pi+0.2])
ylim([0 0.3])
%plot(rot_z_gt,rot_z_gt)
%plot(rot_z_gt,rot_z_gt + 0.2,'r:')
%plot(rot_z_gt,rot_z_gt - 0.2,'r:')
hold off
saveas(gcf,['xyz_translation_' char(name)],'epsc')
%% "Estimated" pose-validation vs real validation
est_valid_pct = sum(reshape(is_valid, [10, rots]))*10;

real_is_valid = is_valid & abs(rot_z_error_corrected) < 0.2 & abs(xyz_error_euclidian) < 0.03;
real_valid_poses = sum(real_is_valid);
real_valid_pct = sum(reshape(real_is_valid, [10, rots]))*10;

y = [real_valid_pct; est_valid_pct-real_valid_pct]';
figure('Name',['histogram_' char(name)])
b = bar(rotations, y,'stacked');
set(b,{'FaceColor'},{'g';'r'});
legend({'True positives', 'False positives'}, 'Location', 'northwest')
ylim([0 100])
xlabel('Actual rotation [rad]')
ylabel('Detections [%]')

saveas(gcf,['histogram_' char(name)],'epsc')
%real_is_valid_all = [real_is_valid_all; real_is_valid];
%% global alignment (+ local alignment)
%% z rotation
rot_z_global = data{5:32:end,2}';
rot_z_global_error = rot_z_global-rot_z_gt;
rot_z_global_error_correction = (rot_z_global_error > pi)*-2*pi + (rot_z_global_error < -pi)*2*pi;
rot_z_global_error_corrected = rot_z_global_error + rot_z_global_error_correction;
rot_z_global_corrected = rot_z_gt+rot_z_global_error_corrected;

figure('Name',['z_rotation_refinement_' char(name)])
plot(rot_z_global_error_corrected(real_is_valid))
hold on
plot(rot_z_error_corrected(real_is_valid))
legend({'z-rotation before refinement', 'z-rotation after refinement'}, 'Location', 'northwest')
xlabel('Pose estimations within threshold')
ylabel('Estimated rotation [rad]')
%% xyz translation
xyz_global = [data{1:32:end,5},data{2:32:end,5},data{3:32:end,5}]';
xyz_global_error_euclidian = sqrt(sum((xyz_global-xyz_gt).^2, 1));
xyz_global_valid = xyz_global_error_euclidian(real_is_valid);
%
figure('Name',['xyz_translation_refinement_' char(name)])
plot(xyz_global_valid)
hold on
plot(xyz_error_euclidian(real_is_valid))
legend({'Translation offset before refinement', 'Translation offset after refinement'}, 'Location', 'northwest')
xlabel('Pose estimations within threshold')
ylabel('Distance from actual xyz-position [m]')
%% Time
global_time_surface_normals = data{6:32:end,2}';
global_time_shape_features = data{7:32:end,2}';
global_time_feature_matches = data{8:32:end,2}';
global_time_ransac = data{9:32:end,2}';

global_ransac_iterations = data{10:32:end,2}';
global_ransac_inliers = data{11:32:end,2}';
global_features = data{12:32:end,2}';
global_object_cloud_size = data{13:32:end,2}';
global_scene_cloud_size = data{14:32:end,2}';
global_rms_error = data{15:32:end,2}';

local_time_icp = data{21:32:end,2}';

local_icp_inliers = data{22:32:end,2}';
local_object_cloud_size = data{23:32:end,2}';
local_scene_cloud_size = data{24:32:end,2}';
local_rms_error = data{25:32:end,2}';

global_time = [ global_time_surface_normals;
                global_time_shape_features;
                global_time_feature_matches;
                global_time_ransac];
            
time = [global_time; local_time_icp]';
%figure('Name',['bar_plot_global_local_' char(name)])
%b = bar(time,'stacked');

%figure
%plot(global_ransac_iterations)
%hold on
%plot(global_ransac_inliers)
%plot(global_features)

disp(name)
avg = mean(time)

%figure
%plot(avg)

%set(b,{'FaceColor'},{'g';'r'});
%legend({'True positives', 'False positives'}, 'Location', 'northwest')
%ylim([0 100])
%xlabel('Actual rotation [rad]')
%ylabel('Detections [%]')

%break

end
%%

