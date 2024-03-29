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

% Kp = [13;17;20];



% Write code here
if question ~=6
% Kp = [13;17;20];
% Kd = [6.6;6.6;9];
Kp = [13;30;20];
Kd = [6.6;10;9];

% Kp = [25;60;200];
% Kd = [10;15;100];



pos_error = current_state.pos - desired_state.pos;
vel_error = current_state.vel - desired_state.vel;

e_ddot = -Kp.*pos_error - Kd.*vel_error;

F = m*(g+desired_state.acc(3)+e_ddot(3));

acc = desired_state.acc + e_ddot;



    
end
end
