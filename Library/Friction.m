function F = Friction(q_1d, q_2d, q_3d, q_4d, Tc1, Tc2, Tc3, Tc4)

F=[- 1.0*Tc1 - 374.0*q_1d;
   - 1.0*Tc2 - 374.0*q_2d;
   - 1.0*Tc3 - 149.0*q_3d;
   - 1.0*Tc4 - 238.0*q_4d];
end