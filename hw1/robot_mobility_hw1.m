%% Q1.3
format short
theta = 30;
m = 100;
a =[];
g=9.8;
h = linspace(0.5,2,5)
lf = linspace(0.25,1.75,100)

legend_list = []
for j=1:size(h,2)
%      legend_list(j,:)=sprintf('h = %s',string(h(j)));
    for i=1:size(lf,2)
        a(j,i) = g*cos(theta)*lf(i)/h(j)-g*sin(theta);
    end
end
figure();
legendCell = cellstr(num2str(h', 'h=%-0.5f'))

for j=1:size(h,2)
    plot(lf,a(j,:)); hold on;
    xlabel('lf [m]')
    ylabel('Acceleration [m/s^2]')
    legend(legendCell)
end

%% Q4.4
x0 = [0;0;0]
time = [0:0.01:20];
global v lf lr df;
v = 20;
lf = 1.5;
lr = 1.5;

%% Part A
df_consts=[1,1.2,1.3,1.4,1.5,1.6];
figure();
for i=1:size(df_consts,2)
    df = df_consts(i);
    [t,xt] = ode45(@pepy_ss,time,x0);
    plot(xt(:,1) , xt(:,2));hold on;
    xlabel('x[m]');
    ylabel('y[m]');
    legend('df = 1','df = 1.2','df = 1.3','df = 1.4','df = 1.5','df = 1.6' )
end

%% Part B
%different amp 
amps=[0.05,0.06,0.07,0.08,0.09];
figure();
for i=1:size(amps,2)
    amp = amps(i);
    global amp;
    [t,xt] = ode45(@pepy_ss,time,x0);
    plot(xt(:,1) , xt(:,2));hold on;
    xlabel('x[m]');
    ylabel('y[m]');
    legend('amplitude = 0.05','amplitude = 0.06','amplitude = 0.07','amplitude = 0.08','amplitude = 0.09' )
end

vs=[30,35,40,45,50];
figure();
for i=1:size(vs,2)
    v=vs(i);
    amp = amps(5);
%     amp = amps(i);
%     global amp;
%     df = amp(i)*square(1/20*2*pi*t);
    [t,xt] = ode45(@pepy_ss,time,x0);
    plot(xt(:,1) , xt(:,2));hold on;
    xlabel('x[m]');
    ylabel('y[m]');
    legend('vx = 30','vx = 35','vx = 40','vx = 45','vx = 50' )
end


%% Part C 
amps=[0.5,0.6,0.7,0.8,0.9];
figure();
v=30;
for i=1:size(amps,2)
    amp = amps(i);
    global amp;
    [t,xt] = ode45(@pepy_ss,time,x0);
    plot(xt(:,1) , xt(:,2));hold on;
    xlabel('x[m]');
    ylabel('y[m]');
    legend('amplitude = 0.5','amplitude = 0.6','amplitude = 0.7','amplitude = 0.8','amplitude = 0.9' )
end



%% Q5.1
global Vx m Iz lf lr caf car A1 A2 B1 B2;
Vx = 30;
m = 1573;
Iz = 2873;
lf = 1.1;
lr = 1.58;
caf = 80000;
car = 80000;
x0=[0 ;0 ;0 ;0];


A = [0,1,0,0;
     0, -(2*caf+2*car)/(m*Vx) , (2*caf+2*car)/m, (-2*caf*lf+2*car*lr)/(m*Vx);
     0,0,0,1;
     0,-(2*caf*lf-2*car*lr)/(Iz*Vx), (2*caf*lf-2*car*lr)/(Iz), -(2*caf*lf^2+2*car*lr^2)/(Iz*Vx)
    ]

B1 = [0;
     2*caf/m;
     0;
     2*caf*lf/Iz;
    ]

B2 =[0;
     -(2*caf*lf-2*car*lr)/(m*Vx)-Vx;
     0;
     -(2*caf*lf^2+2*car*lr^2)/(Iz*Vx);
    ]

Q = [500,0,0,0;0,5,0,0;0,0,500,0;0,0,0,5];
R = 1
[K,S,P] = lqr(A,B1,Q,R);

A2 = A - B1*K;

C= [1 0 0 0;
    0 0 1 0];
D = 0;
sys = ss(A2,B2,C,D);




%% 5.2
desired_ang = atan2(5,90);
time1 = linspace(0,5,500);
si_dot1=desired_ang/2*ones(2,1);
si_dot2 =zeros(300,1);
si_dot3=-desired_ang/2*ones(2,1);
si_dot4=zeros(33,1);
si_dot5 = zeros(163,1)
si_dot = vertcat(si_dot1, si_dot2, si_dot3,si_dot4,si_dot5);

[y,t,x]=lsim(sys,si_dot,time1)
plot(time1, x(:,1));hold on;
plot(time1, x(:,3)); 
xlabel('time[s]')
ylabel('error')
legend('e1','e2')


%% 5.3
si1 = [desired_ang/2,desired_ang]'
si2 = desired_ang * ones(300,1);
si3 = [desired_ang/2,0]'
si4 = zeros(33,1);
si= vertcat(si1 , si2, si3, si4)
si_actual= si+x(1:337,3)

% si = cumtrapz(time1(:,1:337),si_dot(1:337,:));
% new_si= si(1:337,:);
x_dot = Vx*cos(si);
y_dot = Vx*sin(si);
desired_x = cumtrapz(time1(:,1:337),x_dot);
desired_y =cumtrapz(time1(:,1:337),y_dot);
figure();
plot(desired_x,desired_y);hold on;

x_dot_actual = Vx*cos(si_actual);
y_dot_actual = Vx*sin(si_actual);
actual_x = cumtrapz(time1(:,1:337),x_dot_actual);
actual_y =cumtrapz(time1(:,1:337),y_dot_actual);
plot(actual_x, actual_y);
legend('Desired Path', 'Actual Path')
xlabel('x[m]')
ylabel('y[m]')
% plot(desired_traj)
% title('Desired Path')


% actual_x = desired_x + x(:,1)
% figure();
% plot(time1,actual_x);
% title('Actual Path');



%% 5.4
R1 = 1000;
R2=500;
si_dot_val1 = Vx / R1; 
si_dot_val2 = Vx / R2; 
time2 = linspace(0,12,1200);
si_dot1=zeros(100,1);
si_dot2 = si_dot_val1*ones(500,1);
si_dot3=zeros(100,1);
si_dot4=-si_dot_val2*ones(500,1);
si_dot = vertcat(si_dot1, si_dot2, si_dot3, si_dot4);

[y,t,x]=lsim(sys,si_dot,time2);
plot(time2, x(:,1));hold on;
plot(time2, x(:,3)); 
% [t,xt] = ode45(@SS_lane_change,time1,x0);
% plot(time1,xt(1,:));


%% 5.5
si = cumtrapz(time2,si_dot);
x_dot = Vx*cos(si);
y_dot = Vx*sin(si);
desired_x = cumtrapz(time2,x_dot);
desired_y =cumtrapz(time2,y_dot);
figure();
plot(desired_x,desired_y);
title('Desired Path')
xlabel('x[m]');
ylabel('y[m]');

si_actual = si+x(:,3)
x_dot_actual = Vx*cos(si_actual);
y_dot_actual = Vx*sin(si_actual);
actual_x = cumtrapz(time2,x_dot_actual);
actual_y =cumtrapz(time2,y_dot_actual);
actual_x = desired_x + x(:,1)
figure();
plot(actual_x,actual_y);
title('Actual Path');
xlabel('x[m]');
ylabel('y[m]');

%% 5.6
R1 = 1000;
R2=500;
Vx_list = [30 , 60]
figure()
vx = Vx_list(1)
si_dot_val1 = vx / R1; 
si_dot_val2 = vx / R2; 
time2 = linspace(0,12,1200);
si_dot1=zeros(100,1);
si_dot2 = si_dot_val1*ones(500,1);
si_dot3=zeros(100,1);
si_dot4=-si_dot_val2*ones(500,1);
si_dot = vertcat(si_dot1, si_dot2, si_dot3, si_dot4);
[y,t,x]=lsim(sys,si_dot,time2);
plot(time2, x(:,1));hold on;
plot(time2, x(:,3)); 

vx = Vx_list(2)
si_dot_val1 = vx / R1; 
si_dot_val2 = vx / R2; 
time2 = linspace(0,12,1200);
si_dot1=zeros(100,1);
si_dot2 = si_dot_val1*ones(500,1);
si_dot3=zeros(100,1);
si_dot4=-si_dot_val2*ones(500,1);
si_dot = vertcat(si_dot1, si_dot2, si_dot3, si_dot4);
[y,t,x]=lsim(sys,si_dot,time2);
plot(time2, x(:,1),'+');
plot(time2, x(:,3),'+'); 
legend('e1(Vx = 30)','e2(Vx = 30)','e1(Vx = 60)','e2(Vx = 60)');
xlabel('time[s]')
ylabel('error')
