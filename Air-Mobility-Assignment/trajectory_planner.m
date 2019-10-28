function trajectory_state = trajectory_planner(question, waypoints, max_iter, waypoint_times, time_step)

% Input parameters
% 
%   question: Which question we are on in the assignment
%
%   waypoints: Series of points in [x; y; z; yaw] format
%
%   max_iter: Number of time steps
%
%   waypoint_times: Time we should be at each waypoint
%
%   time_step: Length of one time_step
% 
% Output parameters
%
%   trajectory_sate: [15 x max_iter] output trajectory as a matrix of states:
%   [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
%
%************  TRAJECTORY PLANNER ************************

% Write code here

% Sample code for hover trajectory
trajectory_state = zeros(15,max_iter);
% height of 15 for: [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

current_waypoint_number = 1;


% if question ==3
% peak_velocity =1;
% waypoint_len = size(waypoints,2);
% ramp_up = linspace(0,peak_velocity,round(waypoint_len/3));
% flat = ones(1,round(waypoint_len/3))*peak_velocity;
% ramp_down =linspace(peak_velocity,0,waypoint_len - size(flat,2)-size(ramp_up,2));
% velocity_profile =[ramp_up,flat,ramp_down];
% z_acc = cumtrapz(velocity_profile); 
% end


for iter = 1:max_iter
    
    if (current_waypoint_number<length(waypoint_times))
        if((iter*time_step)>waypoint_times(current_waypoint_number+1))
            current_waypoint_number = current_waypoint_number + 1;
        end
    end
        
    trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number);
    trajectory_state(9,iter) = waypoints(4,current_waypoint_number);
    
    if question == 3
    trajectory_state(6,iter) =  waypoints(5,current_waypoint_number);
    trajectory_state(15,iter) =  waypoints(6,current_waypoint_number);
    
    
    elseif question ==8
    trajectory_state(4,iter) =  waypoints(5,current_waypoint_number);
    trajectory_state(5,iter) =  waypoints(6,current_waypoint_number);
    
    elseif question ==9
    trajectory_state(4,iter) =  waypoints(5,current_waypoint_number);
    trajectory_state(5,iter) =  waypoints(6,current_waypoint_number);
        
        
   
        
    end  
end
end
