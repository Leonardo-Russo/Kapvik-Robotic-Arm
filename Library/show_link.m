function show_link(~)
% Description: this function draws a link in 3D space.

% Parameters for the cylinder
diameter = 2; % example diameter
radius = diameter / 2;
length = 5; % example length
axis_orientation = [1, 0, 0]; % example orientation along the x-axis

% Create the cylinder along the z-axis
[zz, yy, xx] = cylinder(radius, 50); % 100 points around the circumference for smoothness
xx = xx * length; % Scale the length of the cylinder

% Angle of rotation to align the cylinder with the axis orientation
theta = acos(dot(axis_orientation, [0 0 1])/(norm(axis_orientation)*norm([0 0 1]))); % Angle between orientation and z-axis
R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]; % Rotation matrix around y-axis

% Apply the rotation to all points
all_points = [xx(:)'; yy(:)'; zz(:)'];
rotated_points = R * all_points;

% Extract the rotated coordinates
xx_rotated = reshape(rotated_points(1,:), size(xx));
yy_rotated = reshape(rotated_points(2,:), size(yy));
zz_rotated = reshape(rotated_points(3,:), size(zz));

% Plot the cylinder
s = surf(xx_rotated, yy_rotated, zz_rotated, 'EdgeColor', 'none');
hold on

% Calculate the end cap coordinates
end_cap_top_x = xx_rotated(1, :);
end_cap_top_y = yy_rotated(1, :);
end_cap_top_z = zz_rotated(1, :);

end_cap_bottom_x = xx_rotated(2, :);
end_cap_bottom_y = yy_rotated(2, :);
end_cap_bottom_z = zz_rotated(2, :);

% Draw the end caps
top_end = fill3(end_cap_top_x, end_cap_top_y, end_cap_top_z, 'y', 'EdgeColor','none'); % Top circle
bot_end = fill3(end_cap_bottom_x, end_cap_bottom_y, end_cap_bottom_z, 'y', 'EdgeColor','none'); % Bottom circle

axis equal

Body = struct('name', "Test Cylinder");
Body.surfs = [s, top_end, bot_end];

rotate_body(Body, [1, 0, 0], 45)

end

function rotate_body(cylinder, axis, angle)
    
    for i = 1 : length(cylinder.surfs)
        rotate(cylinder.surfs(i), axis, angle);
    end
end