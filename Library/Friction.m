function F = Friction(q_1d, q_2d, q_3d, q_4d, Tc1, Tc2, Tc3, Tc4)

F= [                     - Tc1 - (46746*q_1d)/125;
                      - Tc2 - (46746*q_2d)/125;
- Tc3 - (5238143763469041*q_3d)/35184372088832;
- Tc4 - (4185480127129321*q_4d)/17592186044416];
end