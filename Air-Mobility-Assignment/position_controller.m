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

% Kp = [13;17;20];



% Write code here
if question ~=6
Kp = [26;34;40];
Kd = [6.6;6.6;9];

pos_error = current_state.pos - desired_state.pos;
vel_error = current_state.vel - desired_state.vel;

e_ddot = -Kp.*pos_error - Kd.*vel_error;

F = m*(g+desired_state.acc(3)+e_ddot(3));

acc = desired_state.acc + e_ddot;

elseif question ==6
m = params.mass;
J = params.inertia;
J11 = J(1,1);
J22 = J(2,2);
J33 = J(3,3);
psi = current_state.rot(3);
psi_d = desired_state.omega(3);




A = [0,0,0,0,0,0,1,0,0,0,0,0;
     0,0,0,0,0,0,0,1,0,0,0,0;
     0,0,0,0,0,0,0,0,1,0,0,0;
     0,0,0,0,0,0,0,0,0,1,0,0;
     0,0,0,0,0,0,0,0,0,0,1,0;
     0,0,0,0,0,0,0,0,0,0,0,1;
     0,0,0,g*sin(psi),g*cos(psi),0,0,0,0,0,0,0;
     0,0,0,-g*cos(psi),g*sin(psi),0,0,0,0,0,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     0,0,0,(J22-J33)*psi_d^2/J11,0,0,0,0,0,0,(J22-J33)*psi_d/J11,0;
     0,0,0,0,(J11-J33)*psi_d^2/J22,0,0,0,0,-(J11-J33)*psi_d/J22,0,0;
     0,0,0,0,0,0,0,0,0,0,0,0;
     ];

B = [0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     1/m,0,0,0;
     0,1/J11,0,0;
     0,0,1/J22,0;
     0,0,0,1/J33;];
 
 Q = 0.1 * eye(12);
 R = 2*eye(4);
 
K = lqr(A,B, Q,R);
    
end
end
