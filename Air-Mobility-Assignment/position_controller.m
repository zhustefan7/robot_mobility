function [F, acc] = position_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
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
%   question: Question number
%
% Output parameters
%
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% Example PD gains

% % gain for error in x 
% Kp1 = 13;
% Kd1 = 6.6;
% 
% %gain for error in y
% Kp2 = 17;
% Kd2 = 6.6;
% 
% Kp3 = 20;
% Kd3 = 9;
m = params.mass;
g = params.gravity;
b = [0,0,1]; 

Kp = [13;17;20];
Kd = [6.6;6.6;9];

% Write code here
% if question ==2
pos_error = current_state.pos - desired_state.pos
vel_error = current_state.vel - desired_state.vel

e_ddot = -Kp.*pos_error - Kd.*vel_error;


% ex_ddot = - Kp1*pos_error(1)-Kd1*vel_error(1);
% ey_ddot = - Kp1*pos_error(2)-Kd1*vel_error(2);
% ez_ddot = -pos_error(3) - vel_error(3);
% 

F = m*(g+desired_state.acc(3) - Kp(3)*pos_error(3)-Kd(3)*vel_error(3));

acc = desired_state.acc + e_ddot;


% acc = [desired_state.acc(1)+ex_ddot,
%        desired_state.acc(2)+ey_ddot,
%        desired_state.acc(3)+ez_ddot]

% end 
end
