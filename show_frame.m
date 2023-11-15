function show_frame(X, color)
% Description: this function plots the reference frames from the pose.

if nargin < 2
    color = 'c';
end

r = X(1:3);
roll = X(4);
pitch = X(5);
yaw = X(6);

F = R1(roll) * R2(pitch) * R3(yaw) * 0.1*eye(3);

plot3(r(1), r(2), r(3), 'Color', color, 'marker', '*')
hold on

quiver3(r(1), r(2), r(3), F(1, 1), F(2, 1), F(3, 1), "Color", color, 'LineWidth', 1.5)
quiver3(r(1), r(2), r(3), F(1, 2), F(2, 2), F(3, 2), "Color", color, 'LineWidth', 1.5)
quiver3(r(1), r(2), r(3), F(1, 3), F(2, 3), F(3, 3), "Color", color, 'LineWidth', 1.5)

text(r(1) + F(1, 1), r(2) + F(2, 1), r(3) + F(3, 1), 'X', 'Color', color)
text(r(1) + F(1, 2), r(2) + F(2, 2), r(3) + F(3, 2), 'Y', 'Color', color)
text(r(1) + F(1, 3), r(2) + F(2, 3), r(3) + F(3, 3), 'Z', 'Color', color)

end