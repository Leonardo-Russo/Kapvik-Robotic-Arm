function [T_12S, T_22S, T_32S, T_W2S, T_T2S, X_T] = ...
          getManipulatorState(Q, TableMDHsym, X_Tsym, T_B2S, T_T2W)
% Description: this function evaluates the manipulator state from its
% joint variables state Q.

global Qsym
q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

% Compute the Symbolic Quantities
TableMDH = double(subs(TableMDHsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
X_T = double(subs(X_Tsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

% Compute Transformation Matrices
T_12B = tableRow2T(TableMDH(1, :));
T_12S = T_B2S * T_12B;
T_221 = tableRow2T(TableMDH(2, :));
T_22S = T_B2S * T_12B * T_221;
T_322 = tableRow2T(TableMDH(3, :));
T_32S = T_B2S * T_12B * T_221 * T_322;
T_W23 = tableRow2T(TableMDH(4, :));
T_W2S = T_B2S * T_12B * T_221 * T_322 * T_W23;
T_T2S = T_B2S * T_12B * T_221 * T_322 * T_W23 * T_T2W;

end