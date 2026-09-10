clear; clc; close all;

 
urdf_path = 'C:\Users\jonat\Downloads\Compressed\arm_assembly_copied4testing\arm_assembly_copied4testing.urdf';
robot = importrobot(urdf_path);
robot.DataFormat = 'struct';

tipBody = rigidBody('tool_tip');
tipJoint = rigidBodyJoint('tip_joint', 'fixed');
setFixedTransform(tipJoint, trvec2tform([0, 0, 0.025]));
tipBody.Joint = tipJoint;
addBody(robot, tipBody, 'end_effector-v5');

f = figure('Name', 'Kontrol Manual URDF (Cari L-Shape)', 'Position', [100 100 900 600], 'Color', 'w');

ax = axes('Parent', f, 'Position', [0.35 0.1 0.6 0.8]);
show(robot, homeConfiguration(robot), 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
view(ax, 45, 30);
axis(ax, [-0.4 0.4 -0.4 0.4 -0.05 0.5]);
grid on;

hold(ax, 'on');
patch(ax, [-0.4 0.4 0.4 -0.4], [-0.4 -0.4 0.4 0.4], [0 0 0 0], 'green', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
hold(ax, 'off');

title(ax, 'ARM VIRTUAL WORKSPACE');

config = homeConfiguration(robot);

setappdata(f, 'robot', robot);
setappdata(f, 'config', config);
setappdata(f, 'ax', ax);
setappdata(f, 'is_resetting', false);

y_pos = 500;
num_joints = length(config);
labels = {config.JointName};

% Pre-alokasi array handle
h_slider = gobjects(1, num_joints);
h_val = gobjects(1, num_joints);

min_deg = [-179.91, -70.00, -0.36, -357.30, -72.00];
max_deg = [179.91, 100.00, 183.50, 0.59, 72.00];

friendly_labels = {'ID_BASE (ID 1)', 'ID_BAHU (ID 2)', 'ID_SIKU2 (ID 18)', 'ID_WRIST (ID 3)', 'ID_POINTER (ID 15)'};

for i = 1:num_joints
    uicontrol('Style', 'text', 'Position', [20, y_pos, 100, 20], ...
              'String', friendly_labels{i}, 'BackgroundColor', 'w', 'FontWeight', 'bold');
    h_val(i) = uicontrol('Style', 'text', 'Position', [250, y_pos, 60, 20], ...
                         'String', '0.0°', 'BackgroundColor', 'w');
    h_slider(i) = uicontrol('Style', 'slider', 'Position', [120, y_pos, 120, 20], ...
                            'Min', min_deg(i)*(pi/180), 'Max', max_deg(i)*(pi/180), 'Value', 0);
    y_pos = y_pos - 40;
end

% Pasang listener setelah SEMUA slider selesai dibuat agar tidak error indeks
for i = 1:num_joints
    addlistener(h_slider(i), 'Value', 'PostSet', @(src, event) updateRobot(f, h_slider, h_val, num_joints));
end

uicontrol('Style', 'pushbutton', 'Position', [120, y_pos-10, 120, 30], ...
          'String', 'Reset ke 0', 'Callback', @(src, event) resetRobot(f, h_slider, h_val, num_joints));
h_coord = uicontrol('Style', 'text', 'Position', [20, y_pos-80, 200, 60], ...
                    'String', 'Koordinat EE: Menghitung...', ...
                    'BackgroundColor', '#E0F7FA', 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'left');
setappdata(f, 'h_coord', h_coord);

% === PANEL INVERSE KINEMATICS (XYZ) ===
y_ik = y_pos - 150;
uicontrol('Style', 'text', 'Position', [20, y_ik, 60, 20], 'String', 'Target X:', 'BackgroundColor', 'w', 'FontWeight', 'bold');
h_ik_x = uicontrol('Style', 'edit', 'Position', [80, y_ik, 60, 20], 'String', '0.15');

uicontrol('Style', 'text', 'Position', [20, y_ik-30, 60, 20], 'String', 'Target Y:', 'BackgroundColor', 'w', 'FontWeight', 'bold');
h_ik_y = uicontrol('Style', 'edit', 'Position', [80, y_ik-30, 60, 20], 'String', '0.0');

uicontrol('Style', 'text', 'Position', [20, y_ik-60, 60, 20], 'String', 'Target Z:', 'BackgroundColor', 'w', 'FontWeight', 'bold');
h_ik_z = uicontrol('Style', 'edit', 'Position', [80, y_ik-60, 60, 20], 'String', '0.10');

uicontrol('Style', 'pushbutton', 'Position', [20, y_ik-100, 120, 30], 'String', 'Jalankan IK', 'FontWeight', 'bold', 'BackgroundColor', '#4CAF50', 'ForegroundColor', 'w', ...
    'Callback', @(src, event) runIK(f, h_ik_x, h_ik_y, h_ik_z, h_slider, h_val, num_joints, friendly_labels));

function updateRobot(f, h_slider, h_val, num_joints)
    if ~isvalid(f)
        return;
    end
    
    is_res = getappdata(f, 'is_resetting');
    if ~isempty(is_res) && is_res
        return;
    end
    
    robot = getappdata(f, 'robot');
    config = getappdata(f, 'config');
    ax = getappdata(f, 'ax');
    h_coord = getappdata(f, 'h_coord');
    
    mid_deg = [180.00, 100.00, 1.50, 358.00, 72.00];
    
    for i = 1:num_joints
        val = get(h_slider(i), 'Value');

        if i == 2  % ID_BAHU -- arah rotasi URDF terbalik dari servo fisik
            config(i).JointPosition = -val;
        else
            config(i).JointPosition = val;
        end

        % Tampilkan derajat fisik absolut (relatif + titik tengah) -- TETAP pakai val asli, bukan yang dibalik
        absolute_deg = (val * 180/pi) + mid_deg(i);
        set(h_val(i), 'String', sprintf('%.1f°', absolute_deg));
    end
    
    tform_ee = getTransform(robot, config, 'tool_tip');
    tform_shoulder = getTransform(robot, config, 'link_2_V2-v5');
    tform_elbow = getTransform(robot, config, 'link_3-v10');
    tform_wrist = getTransform(robot, config, 'end_effector-v5');
    pos = tform_ee(1:3, 4);
    z_shoulder = tform_shoulder(3, 4);
    z_elbow = tform_elbow(3, 4);
    z_wrist = tform_wrist(3, 4);
    
    % Deteksi nabrak tanah (asumsi ketebalan fisik casing ~ 2.5 cm / 0.025 m)
    is_nabrak = (pos(3) < 0.025) || (z_elbow < 0.025) || (z_wrist < 0.025) || (z_shoulder < 0.025);
    
    if is_nabrak
        coord_text = {'AWAS NABRAK MEJA!', ...
                      sprintf('X = %.5f', pos(1)), ...
                      sprintf('Y = %.5f', pos(2)), ...
                      sprintf('Z = %.5f', pos(3))};
        set(h_coord, 'BackgroundColor', '#FFCDD2', 'ForegroundColor', '#B71C1C', 'String', coord_text);
    else
        coord_text = {'Koordinat EE (m):', ...
                      sprintf('X = %.5f', pos(1)), ...
                      sprintf('Y = %.5f', pos(2)), ...
                      sprintf('Z = %.5f', pos(3))};
        set(h_coord, 'BackgroundColor', '#E0F7FA', 'ForegroundColor', 'k', 'String', coord_text);
    end
    
    show(robot, config, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    
    hold(ax, 'on');
    scatter3(ax, pos(1), pos(2), pos(3), 300, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    hold(ax, 'off');
    
    view(ax, 45, 30);
    axis(ax, [-0.4 0.4 -0.4 0.4 -0.05 0.5]);
    drawnow limitrate;
end

function resetRobot(f, h_slider, h_val, num_joints)
    if ~isvalid(f)
        return;
    end
    
    setappdata(f, 'is_resetting', true);
    
    for i = 1:num_joints
        if isvalid(h_slider(i))
            set(h_slider(i), 'Value', 0);
        end
    end

    if isvalid(f)
        setappdata(f, 'is_resetting', false);
        updateRobot(f, h_slider, h_val, num_joints);
    end
end

function runIK(f, h_x, h_y, h_z, h_slider, h_val, num_joints, friendly_labels)
    robot = getappdata(f, 'robot');
    config = getappdata(f, 'config');
    
    x = str2double(get(h_x, 'String'));
    y = str2double(get(h_y, 'String'));
    z = str2double(get(h_z, 'String'));
    
    if isnan(x) || isnan(y) || isnan(z)
        errordlg('Input X, Y, Z harus berupa angka!', 'Error Input');
        return;
    end
    
    ik = inverseKinematics('RigidBodyTree', robot);
    
    current_tform = getTransform(robot, config, 'tool_tip');
    
    tform = current_tform;
    tform(1:3, 4) = [x; y; z];
    
    % Berikan bobot kecil (0.1) pada orientasi agar IK menghindari rotasi ekstrem pada wrist
    weights = [0.1 0.1 0.1 1 1 1]; 
    
    for i = 1:num_joints
        val = get(h_slider(i), 'Value');
        if i == 2
            config(i).JointPosition = -val;
        else
            config(i).JointPosition = val;
        end
    end
    
    [configSol, solInfo] = ik('tool_tip', tform, weights, config);
    
    if strcmp(solInfo.Status, 'success') || strcmp(solInfo.Status, 'best available')
        setappdata(f, 'is_resetting', true); 
        was_clamped = false;
        clamped_joints = {};
        for i = 1:num_joints
            val = configSol(i).JointPosition;
            if i == 2
                val = -val;
            end
            min_val = get(h_slider(i), 'Min');
            max_val = get(h_slider(i), 'Max');
            if val < min_val || val > max_val
                was_clamped = true;
                clamped_joints{end+1} = friendly_labels{i}; %#ok<AGROW>
            end
            val = max(min_val, min(max_val, val));
            set(h_slider(i), 'Value', val);
        end
        setappdata(f, 'is_resetting', false);

        updateRobot(f, h_slider, h_val, num_joints);

        if was_clamped
            warndlg(sprintf(['IK menemukan solusi (Error: %.5f), tapi sudut berikut melebihi limit fisik servo:\n\n%s\n\n' ...
                'Posisi end-effector di layar TIDAK persis di target X,Y,Z yang kamu masukkan.'], ...
                solInfo.PoseErrorNorm, strjoin(clamped_joints, ', ')), 'Peringatan: Solusi Terpotong (Clamp)');
        else
            disp(['IK Berhasil! Error: ', num2str(solInfo.PoseErrorNorm)]);
        end
    else
        errordlg('IK gagal mencapai target koordinat!', 'Error IK');
    end
end