function DrawJoint(inner_diameter, outer_diameter, Length, axis_orientation, rotation_angle, P)
% Description: Draws a revolute joint consisting of two cylinders in 3D
% space.
    
    blue = [0 0.4470 0.7410];
    orange = [0.8500, 0.3250, 0.0980];

    % Create the inner cylinder (fixed part) and its end caps
    [~, ~, ~, caps_inner] = create_cylinder(inner_diameter, Length, axis_orientation, 0, P); % No rotation

    % Create the outer cylinder (rotating part) and its end caps
    [xx_outer, yy_outer, zz_outer, caps_outer] = create_cylinder(outer_diameter, Length, axis_orientation, rotation_angle, P);

    % Plot the Outer Cylinder
    surf(xx_outer, yy_outer, zz_outer, 'FaceColor', orange, 'EdgeColor', 'none', 'HandleVisibility','off');
    hold on
    
    % Plot the End Caps for the Outer Cylinder (hollow)
    draw_caps(caps_outer, orange, caps_inner);
    
    % Plot the End Caps for the Inner Cylinder (solid)
    draw_caps(caps_inner, blue);

    % Set plot properties
    axis equal

end

function [xx, yy, zz, caps] = create_cylinder(diameter, Length, axis_orientation, rotation_angle, P)
    [zz, yy, xx] = cylinder(diameter/2, 50);
    xx = xx * Length;   % scaling for the cylinder length
    % xx along cilinder axis (x axis of station frame)
    % Set Cylinder Orientation
    theta = atan2(norm(cross(axis_orientation, [1 0 0]))/(norm(axis_orientation)*norm([1 0 0])),...
        dot(axis_orientation, [1 0 0])/(norm(axis_orientation)*norm([1 0 0]))); % angolo tra l'asse del cilindro (x) e l'axis_orientation 
    R = rotation_matrix(theta, rotation_angle, axis_orientation);
    all_points = [xx(:)'; yy(:)'; zz(:)'];
    rotated_points = R * all_points;

    if axis_orientation(3)==1 && axis_orientation(2)==0 && axis_orientation(1)==0
        xc=0;
        yc=0;
        zc=Length/2;
    elseif axis_orientation(2)==1 && axis_orientation(1)==0 && axis_orientation(3)==0
        xc=0;
        yc=Length/2;
        zc=0;
    elseif axis_orientation(1)==1 && axis_orientation(2)==0 && axis_orientation(3)==0
        xc=-Length/2;
        yc=0;
        zc=0;
    end
    x0=(P(1)+xc)*ones(2,length(xx(1,:)));
    y0=(P(2)+yc)*ones(2,length(xx(1,:)));
    z0=(P(3)+zc)*ones(2,length(xx(1,:)));
    xx = x0+reshape(rotated_points(1,:), size(xx));
    yy = y0+reshape(rotated_points(2,:), size(yy));
    zz = z0+reshape(rotated_points(3,:), size(zz));

    % End caps
    caps = {xx(1, :), yy(1, :), zz(1, :); xx(2, :), yy(2, :), zz(2, :)};
end

function R = rotation_matrix(theta, rotation_angle, axis_orientation)
    % Compute the rotation matrix
    if axis_orientation(3)==1 && axis_orientation(2)==0 && axis_orientation(1)==0
        R_align = [cos(-theta) 0 -sin(-theta); 0 1 0; sin(-theta) 0 cos(-theta)];
    elseif axis_orientation(2)==1 && axis_orientation(1)==0 && axis_orientation(3)==0
        R_align = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
    elseif axis_orientation(1)==1 && axis_orientation(2)==0 && axis_orientation(3)==0
        R_align = eye(3);
    end
    R_rotate = [1 0 0; 0 cos(rotation_angle) sin(rotation_angle); 0 -sin(rotation_angle) cos(rotation_angle)];
    R = R_align * R_rotate;
end

function draw_caps(caps, color, inner_caps)
    if nargin < 3
        % Draw solid end caps
        fill3(caps{1, 1}, caps{1, 2}, caps{1, 3}, color, 'EdgeColor', 'none', 'HandleVisibility','off');
        fill3(caps{2, 1}, caps{2, 2}, caps{2, 3}, color, 'EdgeColor', 'none', 'HandleVisibility','off');
    else
        % Draw hollow end caps for the outer cylinder
        fill3([caps{1, 1}, fliplr(inner_caps{1, 1})], ...
              [caps{1, 2}, fliplr(inner_caps{1, 2})], ...
              [caps{1, 3}, fliplr(inner_caps{1, 3})], color, 'EdgeColor', 'none', 'HandleVisibility','off');
        plot3(caps{1, 1}, caps{1, 2}, caps{1, 3}, 'Color', 'black', 'LineWidth', 0.5, 'HandleVisibility','off'); % outer edge

        fill3([caps{2, 1}, fliplr(inner_caps{2, 1})], ...
              [caps{2, 2}, fliplr(inner_caps{2, 2})], ...
              [caps{2, 3}, fliplr(inner_caps{2, 3})], color, 'EdgeColor', 'none', 'HandleVisibility','off');
        plot3(caps{2, 1}, caps{2, 2}, caps{2, 3}, 'Color', 'black', 'LineWidth', 0.5, 'HandleVisibility','off'); % outer edge
    end
end





