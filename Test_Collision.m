clear; clc;

urdf_path = 'C:\Users\jonat\Downloads\Compressed\arm_assembly_copied4testing\arm_assembly_copied4testing.urdf';
robot = importrobot(urdf_path);
robot.DataFormat = 'struct';

tipBody = rigidBody('tool_tip');
tipJoint = rigidBodyJoint('tip_joint', 'fixed');
setFixedTransform(tipJoint, trvec2tform([0, 0, 0.025]));
tipBody.Joint = tipJoint;
addBody(robot, tipBody, 'end_effector-v5');

config = homeConfiguration(robot);
num_joints = length(config);

min_deg = [-179.91, -70.00, -0.36, -357.30, -72.00] * (pi/180);
max_deg = [179.91, 100.00, 183.50, 0.59, 72.00] * (pi/180);

num_tests = 30000;
colliding_data = [];

fprintf('Memulai test %d pose acak...\n', num_tests);

for iter = 1:num_tests
    for i = 1:num_joints
        rand_val = min_deg(i) + rand() * (max_deg(i) - min_deg(i));
        config(i).JointPosition = rand_val;
    end
    
    tform_tip = getTransform(robot, config, 'tool_tip');
    tform_shoulder = getTransform(robot, config, 'link_2_V2-v5');
    tform_elbow = getTransform(robot, config, 'link_3-v10');
    tform_wrist = getTransform(robot, config, 'end_effector-v5');
    
    pos = tform_tip(1:3, 4);
    z_shoulder = tform_shoulder(3, 4);
    z_elbow = tform_elbow(3, 4);
    z_wrist = tform_wrist(3, 4);
    
    if (pos(3) < 0.025) || (z_elbow < 0.025) || (z_shoulder < 0.025) || (z_wrist < 0.025)
        colliding_data = [colliding_data; pos(1), pos(2), pos(3)];
    end
end

fprintf('Ditemukan %d pose yang menabrak meja dari %d test.\n', size(colliding_data, 1), num_tests);
csvwrite('C:\Users\jonat\Downloads\Inverse_Kinematics_Biru\collision_data.csv', colliding_data);
fprintf('Data disimpan ke collision_data.csv\n');

f = figure('Visible', 'off');
scatter3(colliding_data(:,1), colliding_data(:,2), colliding_data(:,3), 10, 'r', 'filled');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Sebaran Koordinat TIP End-Effector yang Menabrak Meja');
grid on;
axis equal;
saveas(f, 'C:\Users\jonat\Downloads\Inverse_Kinematics_Biru\collision_plot.png');
fprintf('Plot disimpan ke collision_plot.png\n');