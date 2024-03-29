function dqdt = SS_lane_change(t,x)
global Vx m Iz lf lr caf car A2 B2;
delta = 0;

R = 1000;
si_dot_val = Vx / R; 
time1 = linspace(0,12,1200);
si_dot1=zeros(100,1);
si_dot2 = si_dot_val*ones(500,1);
si_dot3=zeros(100,1);
si_dot4=-si_dot_val*ones(500,1);
si_dot = vertcat(si_dot1, si_dot2, si_dot3, si_dot4);

dqdt = A2*x + B2*si_dot;
end