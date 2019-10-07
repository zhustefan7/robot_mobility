function dqdt = SS(t,x)
global v lf lr df amp;
amp
df = amp*square(1/5*2*pi*t);
% df = 1
dqdt = [v*cos(x(3));v*sin(x(3));v/(lf+lr)*tan(df)];
end
