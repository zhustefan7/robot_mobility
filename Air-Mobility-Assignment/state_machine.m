function [waypoints, waypoint_times] = state_machine(traj,traj_time,take_off_time,hover_time,land_time,step_size)

point_nums = [take_off_time,hover_time,land_time]/step_size;

%take off phase
take_off_point_num = point_nums(1);
hover_height = traj(3,1); %the first z coordinate of the input trajectory 
x_waypoint = zeros(1,take_off_point_num);
y_waypoint = zeros(1,take_off_point_num);
z_waypoint = linspace(0,hover_height,take_off_point_num );
theta_waypoint = zeros(1,point_nums(1));
take_off_waypoints = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];


%hover phase
hover_point_num = point_nums(2);
hover_waypoints_take_off = hover(traj(4,1),traj(3,1),hover_point_num);
hover_waypoints_land = hover(traj(4,end),traj(3,end),hover_point_num);


%land phase 
landing_point_num = point_nums(3);
hover_height = traj(3,end); %the last z coordinate of the input trajectory 
x_waypoint = zeros(1,landing_point_num);
y_waypoint = zeros(1,landing_point_num);
z_waypoint = linspace(hover_height,0,landing_point_num);
theta_waypoint = linspace(traj(4,end),0,landing_point_num);
land_waypoints = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];



waypoints = [take_off_waypoints,hover_waypoints_take_off,traj,hover_waypoints_land,land_waypoints];
total_time = take_off_time + hover_time+traj_time + hover_time + land_time;
total_point_nums = total_time/step_size;  %assuming the interpolation size is uniform across pipeline
waypoint_times = linspace(0,total_time,total_point_nums);

end 


function hover_waypoints = hover(prev_heading,hover_height,point_num) 
x_waypoint = zeros(1,point_num);
y_waypoint = zeros(1,point_num);
z_waypoint = ones(1,point_num)*hover_height;
theta_waypoint = ones(1,point_num)*prev_heading;
hover_waypoints = [x_waypoint;y_waypoint;z_waypoint;theta_waypoint];
end 





