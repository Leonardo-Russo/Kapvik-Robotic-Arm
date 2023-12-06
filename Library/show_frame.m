function frame = show_frame(T, color, name)
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

X = trans2pose(T);
r = X(1:3);
R = T(1:3, 1:3);

alpha = 0.15;   % axis length
F = R' * alpha*eye(3);

origin = plot3(r(1), r(2), r(3), 'Color', color, 'marker', '*', 'HandleVisibility','off');

linewidth = 1.5;
arrowhead = linewidth/2;

% Le componenti dei versori dei sistemi di riferimento dei joint hanno le
% componenti rispetto al riferimento inerziale (station frame, ovvero il
% sistema di riferimento rispetto a cui è fatto il plot) che sono le righe
% della rotazione da riferimento inerziale a riferimento di ogni joint (
% rotazione definita da F.

arw1 = quiver3(r(1), r(2), r(3), F(1, 1), F(1, 2), F(1, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead);
arw2 = quiver3(r(1), r(2), r(3), F(2, 1), F(2, 2), F(2, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead, 'HandleVisibility','off');
arw3 = quiver3(r(1), r(2), r(3), F(3, 1), F(3, 2), F(3, 3), "Color", color, 'LineWidth', linewidth, 'MaxHeadSize', arrowhead, 'HandleVisibility','off');

txt1 = text(r(1) + F(1, 1), r(2) + F(1, 2), r(3) + F(1, 3), ax1, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex');
txt2 = text(r(1) + F(2, 1), r(2) + F(2, 2), r(3) + F(2, 3), ax2, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex');
txt3 = text(r(1) + F(3, 1), r(2) + F(3, 2), r(3) + F(3, 3), ax3, 'Color', color, 'FontSize', 11, 'Interpreter', 'latex');

frame = struct('name', name);
frame.O = origin;
frame.arw1 = arw1;
frame.arw2 = arw2;
frame.arw3 = arw3;
frame.txt1 = txt1;
frame.txt2 = txt2;
frame.txt3 = txt3;

end