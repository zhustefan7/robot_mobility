function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here
g = params.gravity;
si_d = desired_state.rot(3);
x_ddt_d = desired_state.acc(1);
y_ddt_d = desired_state.acc(2);

rot_d = 1/g*[sin(si_d),-cos(si_d);cos(si_d),sin(si_d)]*[x_ddt_d;y_ddt_d]; %first entry: phi_d. Second entry: theta_d
rot = [rot_d(1);rot_d(2);si_d];

si_dt_d = desired_state.omega(3);
omega_d = 1/g*[cos(si_d),sin(si_d);-sin(si_d),cos(si_d)]*[x_ddt_d*si_dt_d;y_ddt_d*si_dt_d];
omega = [omega_d(1); omega_d(2); si_dt_d];


end

