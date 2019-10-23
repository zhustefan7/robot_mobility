function [waypoints, waypoint_times] = lookup_waypoints(question)
%
% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoings, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

% Write code here

%Sample waypoints for hover trajectory
step_size = 0.01;
if question == 2
   waypoints = [0 0.1 0.2 0.3;
                0 0 0 0;
                0.5 0.5 0.5 0.5; 
                0 0 0 0];
%     waypoints = [0,0,0.1,0;
%                  0,0,0.2,0;
%                  0,0,0.3,0;
%                  0,0,0.5,0;]
    waypoint_times = [0 2 4 6];

elseif question ==3
    point_num = 100;
    x_waypoint = zeros(1,point_num);
    y_waypoint = zeros(1,point_num);
    z_waypoint = linspace(0,1,point_num);
    theta_waypoint = zeros(1,point_num);
    
    waypoints = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];
%     size(waypoints)
    waypoint_times = linspace(0,10,point_num);
    
elseif question == 5
    take_off_time = 1;
    hover_time = 2;
    traj_time = 3;
    land_time = 2; 
    point_num = traj_time/step_size;
    
    x_waypoint = zeros(1,point_num);
    y_waypoint = zeros(1,point_num);
    z_waypoint = ones(1,point_num)*0.1;
    theta_waypoint = zeros(1,point_num);
    
    traj = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];   
    [waypoints, waypoint_times] = state_machine(traj,traj_time,take_off_time,hover_time,land_time,step_size);
    
  
end
end


% function [waypoints, waypoint_times] = generate_waypoints(start_pos,end_pos,end_time)
% 
% 
% end
