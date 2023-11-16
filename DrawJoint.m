function DrawJoint(inner_diameter, outer_diameter, length, axis_orientation, rotation_angle)
% Description: Draws a revolute joint consisting of two cylinders in 3D
% space.
    
    blue = [0 0.4470 0.7410];
    orange = [0.8500, 0.3250, 0.0980];

    % Create the inner cylinder (fixed part) and its end caps
    [~, ~, ~, caps_inner] = create_cylinder(inner_diameter, length, axis_orientation, 0); % No rotation

    % Create the outer cylinder (rotating part) and its end caps
    [xx_outer, yy_outer, zz_outer, caps_outer] = create_cylinder(outer_diameter, length, axis_orientation, rotation_angle);

    % Plot the Outer Cylinder
    surf(xx_outer, yy_outer, zz_outer, 'FaceColor', orange, 'EdgeColor', 'none');
    hold on
    
    % Plot the End Caps for the Outer Cylinder (hollow)
    draw_caps(caps_outer, orange, caps_inner);
    
    % Plot the End Caps for the Inner Cylinder (solid)
    draw_caps(caps_inner, blue);

    % Set plot properties
    axis equal

end

function [xx, yy, zz, caps] = create_cylinder(diameter, length, axis_orientation, rotation_angle)
    
    [zz, yy, xx] = cylinder(diameter/2, 50);
    xx = xx * length;   % scaling for the cylinder length

    % Set Cylinder Orientation
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

function draw_caps(caps, color, inner_caps)
    if nargin < 3
        % Draw solid end caps
        fill3(caps{1, 1}, caps{1, 2}, caps{1, 3}, color, 'EdgeColor', 'none');
        fill3(caps{2, 1}, caps{2, 2}, caps{2, 3}, color, 'EdgeColor', 'none');
    else
        % Draw hollow end caps for the outer cylinder
        fill3([caps{1, 1}, fliplr(inner_caps{1, 1})], ...
              [caps{1, 2}, fliplr(inner_caps{1, 2})], ...
              [caps{1, 3}, fliplr(inner_caps{1, 3})], color, 'EdgeColor', 'none');
        plot3(caps{1, 1}, caps{1, 2}, caps{1, 3}, 'Color', 'black', 'LineWidth', 0.5); % outer edge

        fill3([caps{2, 1}, fliplr(inner_caps{2, 1})], ...
              [caps{2, 2}, fliplr(inner_caps{2, 2})], ...
              [caps{2, 3}, fliplr(inner_caps{2, 3})], color, 'EdgeColor', 'none');
        plot3(caps{2, 1}, caps{2, 2}, caps{2, 3}, 'Color', 'black', 'LineWidth', 0.5); % outer edge
    end
end





