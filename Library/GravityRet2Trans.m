function G = GravityRet2Trans(q_1, q_2, q_3, q_4)
g=3.71;

G =g*[ 
                                                                                                                                                                                                                                                                  0;
(49648703346548164333338714632765*cos(q_2 + q_3 + q_4))/324518553658426726783156020576256 + (87100946683774676375858792392069*cos(q_2 + q_3))/50706024009129176059868128215040 + (42173157172706479584239547545987943*cos(q_2))/16225927682921336339157801028812800;
                                                                                     (49648703346548164333338714632765*cos(q_2 + q_3 + q_4))/324518553658426726783156020576256 + (87100946683774676375858792392069*cos(q_2 + q_3))/50706024009129176059868128215040;
                                                                                                                                                                          (49648703346548164333338714632765*cos(q_2 + q_3 + q_4))/324518553658426726783156020576256];
  
end