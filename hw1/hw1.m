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
