function [t_pos,M1,M2] = Gen_Measurement(initial_pos,vel,Sigma,Sensor)
sigma_r = Sigma.r;
sigma_a = Sigma.a;

x_initial = initial_pos.x;
y_initial = initial_pos.y;
vx = vel.x;
vy = vel.y;
N = 101;

x_pos = zeros(N,1);
y_pos = zeros(N,1);

x_pos(1) = x_initial;
y_pos(1) = y_initial;

for n = 2:N
    x_pos(n) = x_pos(n-1) + vx + randn*Sigma.u;
    y_pos(n) = y_pos(n-1) + vy + randn*Sigma.u;
end
P1 = Sensor.P1;
P2 = Sensor.P2;
r_mea = sqrt((x_pos-P1(1)).^2+(y_pos-P1(2)).^2) + randn*sigma_r;
a_mea = atan((y_pos-P1(2))./(x_pos-P1(1)+realmin)) + ((x_pos-P1(1))<0)*pi + randn*sigma_a;
M1 = [r_mea a_mea];
r_mea = sqrt((x_pos-P2(1)).^2+(y_pos-P2(2)).^2) + randn*sigma_r;
a_mea = atan((y_pos-P2(2))./(x_pos-P2(1)+realmin)) + ((x_pos-P2(1))<0)*pi + randn*sigma_a;
M2 = [r_mea a_mea];
t_pos = [x_pos,y_pos];
end