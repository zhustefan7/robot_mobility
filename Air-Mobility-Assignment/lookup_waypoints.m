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
step_size = 0.005;
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
    total_time = 10;
    point_num = total_time/step_size;
    waypoint_times = linspace(0,10,point_num);
    
    syms a(t) v(t)

    a(t) = piecewise(t<0, 0,t>=0 & t<2, 0.25, t>=2 & t<4,-0.25,t>=4,0);
    v(t) = int(a(t),0,t);
    z(t) = int(v(t) ,0,t);

    
    vel_waypoint = v(waypoint_times);
    acc_waypoint = a(waypoint_times);
%     vel_waypoint = vel_waypoint - min(vel_waypoint(:));
%     vel_waypoint = vel_waypoint ./ max(vel_waypoint(:));
%     
    
    z_waypoint = z(waypoint_times);
%     z_waypoint= z_waypoint - min(z_waypoint(:));
%     z_waypoint = z_waypoint ./ max(z_waypoint(:)); 
    

%    
%     total_time = 10;
%     point_num = total_time/step_size;
    x_waypoint = zeros(1,point_num);
    y_waypoint = zeros(1,point_num);
%     z_waypoint = linspace(0,1,point_num);
    theta_waypoint = zeros(1,point_num);
%     
    waypoints = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint;vel_waypoint;acc_waypoint];
%     waypoint_times = linspace(0,10,point_num);
%     
%     
     
    
elseif question == 5
    take_off_time = 0;
    hover_time = 0;
    traj_time = 10;
    land_time = 2; 
    point_num = traj_time/step_size;
    
    %5.1 
%     theta_waypoint = zeros(1,point_num);
    
    %5.2
    theta_waypoint = [zeros(1,400),0.261799*ones(1,1600)];
    
    x_waypoint = zeros(1,point_num);
    y_waypoint = zeros(1,point_num);
    z_waypoint = [zeros(1,400),ones(1,1600)*0.1];
    traj = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];   
    [waypoints, waypoint_times] = state_machine(traj,traj_time,take_off_time,hover_time,land_time,step_size);
    
elseif question ==7
    take_off_time = 1;
    hover_time = 0;
    traj_time = 4;
    land_time = 2; 
    point_num = traj_time/step_size;
    x_waypoint = zeros(1,point_num);
    y_waypoint = zeros(1,point_num);
    z_waypoint = linspace(1,10,point_num);
    theta_waypoint = zeros(1,point_num);
    traj = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];   
    [waypoints, waypoint_times] = state_machine(traj,traj_time,take_off_time,hover_time,land_time,step_size);
    
    
    
    
    
    
    
    
    
    
    
elseif question == 8
    a = 2; 
    b= 1;
    take_off_time = 0.5;
    hover_time = 0;
    traj_time = 9.42;
    land_time = 2; 
    point_num = traj_time/step_size;
    
    t = linspace(0,traj_time,point_num);
    x_waypoint = a*cos(t-pi/2);
    y_waypoint = b*sin(t-pi/2)+1;
    z_waypoint = ones(1,point_num)*1;
    theta_waypoint = zeros(1,point_num);
    traj = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];   
    [waypoints, waypoint_times] = state_machine(traj,traj_time,take_off_time,hover_time,land_time,step_size);
    
    %part 1
%     x_dot = linspace(0,1,point_num);
% %     y_dot = b/sqrt(a^2+b^2)*sin(t);
%     x_waypoints = zeros(1,size(waypoints,2));
%     y_waypoints = zeros(1,size(waypoints,2));
%     x_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = x_dot;
% %     y_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = y_dot;
%     waypoints = [waypoints;x_waypoints;y_waypoints];
%     
    
    %part 2
    x_dot = a/sqrt(a^2+b^2)*cos(t);
    y_dot = b/sqrt(a^2+b^2)*sin(t);
    
%     x_dot(1,1256:1884) = -x_dot(1,1:629);
%     y_dot(1,1256:1884) = -y_dot(1,1:629);
    
    
    x_dot_waypoints = zeros(1,size(waypoints,2));
    y_dot_waypoints = zeros(1,size(waypoints,2));
    x_dot_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = x_dot;
    y_dot_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = y_dot;
    waypoints = [waypoints;x_dot_waypoints;y_dot_waypoints];
    
    
elseif question == 9
    a = 2; 
    b= 2;
    take_off_time = 1;
    hover_time = 0;
    traj_time = 6.18;
    land_time = 2; 
    point_num = traj_time/step_size;
    t = linspace(0,traj_time,point_num);
    x_waypoint = a*cos(t-pi/2);
    y_waypoint = b*sin(t-pi/2)+1;


    x_dot = a/sqrt(a^2+b^2)*cos(t);
    y_dot = b/sqrt(a^2+b^2)*sin(t);

    initial_heading = atan2(y_dot, x_dot);
    for i=1 : size(initial_heading,2)
        if initial_heading(i) < 0 
            initial_heading(i)= initial_heading(i)+ 2*pi;
        end
    end
    
    theta_waypoint = initial_heading + pi/2;
    z_waypoint = ones(1,point_num)*1;
    traj = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];   
    [waypoints, waypoint_times] = state_machine(traj,traj_time,take_off_time,hover_time,land_time,step_size);
     
    %part 1
%     x_dot = linspace(0,1,point_num);
%     x_dot_waypoints = zeros(1,size(waypoints,2));
%     y_dot_waypoints = zeros(1,size(waypoints,2));
%     x_dot_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = x_dot;
%     waypoints = [waypoints;x_dot_waypoints;y_dot_waypoints];\
    
    
    
     %part 2
    x_dot = a/sqrt(a^2+b^2)*cos(t);
    y_dot = b/sqrt(a^2+b^2)*sin(t);
   
    
    x_dot_waypoints = zeros(1,size(waypoints,2));
    y_dot_waypoints = zeros(1,size(waypoints,2));
    x_dot_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = x_dot;
    y_dot_waypoints(1, (take_off_time+hover_time)/step_size+1:(take_off_time+hover_time+traj_time)/step_size) = y_dot;
    waypoints = [waypoints;x_dot_waypoints;y_dot_waypoints];
    
    
        
    
end
end


% function [waypoints, waypoint_times] = generate_waypoints(start_pos,end_pos,end_time)
% 
% 
% end
