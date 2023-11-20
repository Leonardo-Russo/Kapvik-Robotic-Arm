function DrawJoint(inner_diameter, outer_diameter, Length, T)
% Description: Draws a revolute joint consisting of two cylinders in 3D
% space.
    
    blue = [0 0.4470 0.7410];
    orange = [0.8500, 0.3250, 0.0980];

    % Create the inner cylinder (fixed part) and its end caps
    [~, ~, ~, caps_inner] = create_cylinder(inner_diameter, Length, T); % No rotation

    % Create the outer cylinder (rotating part) and its end caps
    [xx_outer, yy_outer, zz_outer, caps_outer] = create_cylinder(outer_diameter, Length, T);

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

function [xx, yy, zz, caps] = create_cylinder(diameter, Length, T)
    [zz, yy, xx] = cylinder(diameter/2, 50);
    xx = xx * Length-Length/2;   % scaling for the cylinder length
    % xx along cilinder axis (x axis of station frame)
    % Retrieve the Pose
    X = trans2pose(T);
    x0 = X(1);
    y0 = X(2);
    z0 = X(3);
    roll = rad2deg(X(4));
    pitch = rad2deg(X(5));
    yaw = rad2deg(X(6));
    % Set Cylinder Orientation
    R = R3(-yaw,"deg")*R2(-pitch,"deg")*R1(-roll,"deg")*R2(90,"deg");
    all_points = [xx(:)'; yy(:)'; zz(:)'];
    rotated_points = R * all_points;
    xx = x0+reshape(rotated_points(1,:), size(xx));
    yy = y0+reshape(rotated_points(2,:), size(yy));
    zz = z0+reshape(rotated_points(3,:), size(zz));

    % End caps
    caps = {xx(1, :), yy(1, :), zz(1, :); xx(2, :), yy(2, :), zz(2, :)};
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





