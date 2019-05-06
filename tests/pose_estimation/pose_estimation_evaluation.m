data = readtable('pose_data_yoshi_cut_off_5_cm.csv','Delimiter',',','ReadVariableNames', false);
xyz = [data{26:32:end,5},data{27:32:end,5},data{28:32:end,5}];

%figure
%plot(xyz)
rot_z = data{30:32:end,2}';
rotations = [linspace(0,-7/8*pi,8) linspace(pi,1/8*pi,8)];
rot_z_gt = repelem(rotations,10);
rot_z_error = rot_z-rot_z_gt;
is_valid = logical(data{31:32:end,2}');
valid_poses = sum(is_valid);

rot_z_error_correction = (rot_z_error > pi)*-2*pi + (rot_z_error < -pi)*2*pi;
rot_z_error_corrected = rot_z_error + rot_z_error_correction;
rot_z_corrected = rot_z_gt+rot_z_error_corrected;

%figure
%plot(rot_z_corrected)
%hold on
%plot(rot_z_gt)

%figure
%plot(rot_z)
%hold on
%plot(rot_z_gt)

figure
scatter(rot_z_gt, rot_z_corrected)
hold on
plot(rot_z_gt,rot_z_gt)
legend({'Estimated vs actual rotation', 'Ground truth'})
xlabel('Actual rotation [rad]')
ylabel('Estimated rotation [rad]')

%figure
%plot(rot_z_error_corrected)
%hold on
%plot(rot_z_error_corrected*is_valid)

%figure
%plot(rot_z)
%hold on
%plot(is_valid)