function show_frame(X, color, name)
% Description: this function plots the reference frames from the pose.

if nargin < 2
    color = 'k';
end

if nargin < 3
    name = "";
end

if name ~= ""
    ax1 = strcat("$X_", name, "$");
    ax2 = strcat("$Y_", name, "$");
    ax3 = strcat("$Z_", name, "$");
else
    ax1 = "$X$";
    ax2 = "$Y$";
    ax3 = "$Z$";
end

r = X(1:3);
roll = X(4);
pitch = X(5);
yaw = X(6);

alpha = 0.15;   % axis length
F = R1(roll) * R2(pitch) * R3(yaw) * alpha*eye(3);

plot3(r(1), r(2), r(3), 'Color', color, 'marker', '*', 'HandleVisibility','off')
hold on

linewidth = 1.5;
arrowhead = linewidth/2;

% Le componenti dei versori dei sistemi di riferimento dei joint hanno le
% componenti rispetto al riferimento inerziale (station frame, ovvero il
% sistema di riferimento rispetto a cui è fatto il plot) che sono le righe
% della rotazione da riferimento inerziale a riferimento di ogni joint (
% rotazione definita da F
quiver3(r(1), r(2), r(3), F(1, 1), F(1, 2), F(1, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead);
quiver3(r(1), r(2), r(3), F(2, 1), F(2, 2), F(2, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead, 'HandleVisibility','off')
quiver3(r(1), r(2), r(3), F(3, 1), F(3, 2), F(3, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead, 'HandleVisibility','off')

text(r(1) + F(1, 1), r(2) + F(1, 2), r(3) + F(1, 3), ax1, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex')
text(r(1) + F(2, 1), r(2) + F(2, 2), r(3) + F(2, 3), ax2, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex')
text(r(1) + F(3, 1), r(2) + F(3, 2), r(3) + F(3, 3), ax3, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex')

% quiver3(r(1), r(2), r(3), F(1, 1), F(2, 1), F(3, 1), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead);
% quiver3(r(1), r(2), r(3), F(1, 2), F(2, 2), F(3, 2), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead, 'HandleVisibility','off')
% quiver3(r(1), r(2), r(3), F(1, 3), F(2, 3), F(3, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead, 'HandleVisibility','off')
% 
% text(r(1) + F(1, 1), r(2) + F(2, 1), r(3) + F(3, 1), ax1, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex')
% text(r(1) + F(1, 2), r(2) + F(2, 2), r(3) + F(3, 2), ax2, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex')
% text(r(1) + F(1, 3), r(2) + F(2, 3), r(3) + F(3, 3), ax3, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex')

xlabel('$X_S$', 'fontsize', 12,'Interpreter', 'latex')
ylabel('$Y_S$', 'fontsize', 12,'Interpreter', 'latex')
zlabel('$Z_S$', 'fontsize', 12,'Interpreter', 'latex')
axis equal
grid on
view(3)
hold on

end