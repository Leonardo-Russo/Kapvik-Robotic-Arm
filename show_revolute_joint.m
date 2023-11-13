function show_revolute_joint(inner_diameter, outer_diameter, length, axis_orientation, rotation_angle)
    % Description: Draws a revolute joint consisting of two cylinders
    
    blue = [0 0.4470 0.7410];
    orange = [0.8500, 0.3250, 0.0980];

    % Create the inner cylinder (fixed part) and its end caps
    [xx_inner, yy_inner, zz_inner, caps_inner] = create_cylinder(inner_diameter, length, axis_orientation, 0); % No rotation

    % Create the outer cylinder (rotating part) and its end caps
    [xx_outer, yy_outer, zz_outer, caps_outer] = create_cylinder(outer_diameter, length, axis_orientation, rotation_angle);

    % Plot the inner cylinder and its end caps
    % s_inner = surf(xx_inner, yy_inner, zz_inner);
    % s_inner.EdgeColor = 'none';

    % Plot the outer cylinder and its end caps
    s_outer = surf(xx_outer, yy_outer, zz_outer, 'FaceColor', [0.8500, 0.3250, 0.0980]);
    s_outer.EdgeColor = 'none';
    hold on;
    draw_caps(caps_outer, orange);

    draw_caps(caps_inner, blue);

    % Set plot properties
    axis equal;
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    title('Revolute Joint');
end

function [xx, yy, zz, caps] = create_cylinder(diameter, length, axis_orientation, rotation_angle)
    
radius = diameter / 2;
    [zz, yy, xx] = cylinder(radius, 50);
    xx = xx * length;

    % Apply rotation
    theta = acos(dot(axis_orientation, [0 0 1])/(norm(axis_orientation)*norm([0 0 1])));
    R = rotation_matrix(theta, rotation_angle);

    all_points = [xx(:)'; yy(:)'; zz(:)'];
    rotated_points = R * all_points;

    xx = reshape(rotated_points(1,:), size(xx));
    yy = reshape(rotated_points(2,:), size(yy));
    zz = reshape(rotated_points(3,:), size(zz));

    % End caps
    caps = {xx(1, :), yy(1, :), zz(1, :); xx(2, :), yy(2, :), zz(2, :)};
end

function R = rotation_matrix(theta, rotation_angle)
    % Compute the rotation matrix
    R_align = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_rotate = [cosd(rotation_angle) -sind(rotation_angle) 0; sind(rotation_angle) cosd(rotation_angle) 0; 0 0 1];
    R = R_rotate * R_align;
end

function draw_caps(caps, color)
    % Draw the end caps
    fill3(caps{1, 1}, caps{1, 2}, caps{1, 3}, color, 'EdgeColor', 'none');
    fill3(caps{2, 1}, caps{2, 2}, caps{2, 3}, color, 'EdgeColor', 'none');
end
