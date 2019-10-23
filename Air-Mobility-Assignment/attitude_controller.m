function [M] = attitude_controller(current_state,desired_state,params,question)

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
%   M: u2 or moment [M1; M2; M3]
%
%************  ATTITUDE CONTROLLER ************************

% Example PD gains
% Kpphi = 190;
% Kdphi = 30;
% 
% Kptheta = 198;
% Kdtheta = 30;
% 
% Kppsi = 80;
% Kdpsi = 17.88;
I = params.inertia;

Kp = [190;198;80];
Kd = [30;30;17.88];

% Write code here
R = current_state.rot;
R_d = desired_state.rot;
eR = R - R_d;
eW = current_state.omega- R'*R_d*desired_state.omega;

M = I*(-Kp.*eR - Kd.*eW);


end

