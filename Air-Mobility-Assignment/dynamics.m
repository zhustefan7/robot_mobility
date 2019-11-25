function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************
% Write code here
m = params.mass;
g = params.gravity;
I = params.inertia;

phi=state(7);
theta = state(8); 
psi = state(9);

Rz = [1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];
Ry = [cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
Rx = [cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1];

% Reb= Rz * Ry *Rx;
% Fb = [0;0;F];
% a = (Reb*Fb - [0;0;mass*gravity])/mass;
a = [g*(theta*cos(psi)+phi*sin(psi));g*(theta*sin(psi)-phi*cos(psi));F/m-g];


alpha = inv(I)*M; 


state_dot = zeros(16,1);
state_dot(1) = state(4);
state_dot(2) = state(5);
state_dot(3) = state(6);
state_dot(4) = a(1);
state_dot(5) = a(2);
state_dot(6) = a(3);
state_dot(7) = state(10);
state_dot(8) = state(11);
state_dot(9) = state(12);
state_dot(10) = alpha(1);
state_dot(11) = alpha(2);
state_dot(12) = alpha(3);
state_dot(13:16) = rpm_motor_dot;


end