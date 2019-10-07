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
time = [0:0.01:200];
global v lf lr df;
v = 20;
lf = 1.5;
lr = 1.5;
% df = square(;
[t,xt] = ode45(@pepy_ss,time,x0);
figure();
plot(xt(:,1) , xt(:,2))



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
time1 = linspace(0,7,700);
si_dot1=desired_ang/100*ones(100,1);
si_dot2 =zeros(500,1);
si_dot3=-desired_ang/100*ones(100,1);
si_dot = vertcat(si_dot1, si_dot2, si_dot3);

[y,t,x]=lsim(sys,si_dot,time1)
plot(time1, x(:,1));hold on;
plot(time1, x(:,3)); 


%% 5.3

si = cumtrapz(time1,si_dot);
x_dot = Vx.*si;
desired_x = cumtrapz(time1,x_dot);
figure();
plot(time1,desired_x);
title('Desired Path')


actual_x = desired_x + x(:,1)
figure();
plot(time1,actual_x);
title('Actual Path');

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
x_dot = Vx.*si;
desired_x = cumtrapz(time2,x_dot);
figure();
plot(time2,desired_x);
title('Desired Path')


actual_x = desired_x + x(:,1)
figure();
plot(time2,actual_x);
title('Actual Path');
