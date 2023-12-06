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