function dqdt = SS(t,x)
global v lf lr df;
df = square(1/100*2*pi*t);
dqdt = [v*cos(x(3));v*sin(x(3));v/(lf+lr)*tan(df)];
end
