%%
clc;
clear;
close all;
%%
Sigma.u = sqrt(0.1);
Sigma.r = 5;
Sigma.a = 0.01/180*pi;
initial_pos.x = 50;
initial_pos.y = -100;
vel.x = -1;
vel.y = 2;
Sensor.P1 = [-100 0];
Sensor.P2 = [ 100 0];
[t_pos,M1,M2] = Gen_Measurement(initial_pos,vel,Sigma,Sensor);

X_f1 = EKF(M1(:,1),M1(:,2),Sigma);
X_f2 = EKF(M2(:,1),M2(:,2),Sigma);
X_fCC = BC_EKF(M1(:,1),M1(:,2),M2(:,1),M2(:,2),Sigma,Sensor);

%%
P1 = Sensor.P1;
P2 = Sensor.P2;
N = size(t_pos,1);
figure; hold on; grid on;
plot(t_pos(:,1),t_pos(:,2),'LineWidth',1);
plot(M1(:,1).*cos(M1(:,2))+P1(1),M1(:,1).*sin(M1(:,2))+P1(2),'LineWidth',1);
plot(M2(:,1).*cos(M2(:,2))+P2(1),M2(:,1).*sin(M2(:,2))+P2(2),'LineWidth',1);
plot(X_f1(:,1)+P1(1),X_f1(:,3)+P1(2),'LineWidth',1);
plot(X_f2(:,1)+P2(1),X_f2(:,3)+P2(2),'LineWidth',1);
plot(X_fCC(:,1),X_fCC(:,3),'LineWidth',1);
legend('true track','measure1 track','measure2 track',...
       'filter1 track','filter2 track','Fusion track');
xlabel('X/m');ylabel('Y/m');

figure; hold on; grid on;
plot(1:101,t_pos(:,1)-X_f1(:,1)-P1(1),'LineWidth',1);
plot(1:101,t_pos(:,1)-X_f2(:,1)-P2(1),'LineWidth',1);
legend('filter1 X Bias','filter2 X Bias');axis tight;
xlabel('t/s');ylabel('Bias/m');

figure; hold on; grid on;
plot(1:101,t_pos(:,2)-X_f1(:,3)-P1(2),'LineWidth',1);
plot(1:101,t_pos(:,2)-X_f2(:,3)-P2(2),'LineWidth',1);
legend('filter1 Y Bias','filter2 Y Bias');axis tight;
xlabel('t/s');ylabel('Bias/m');

figure; hold on; grid on;
plot(1:101,t_pos(:,1)-X_fCC(:,1),'LineWidth',1);
plot(1:101,t_pos(:,2)-X_fCC(:,3),'LineWidth',1);
legend('X Bias','Y Bias');axis tight;
xlabel('t/s');ylabel('Bias/m');
%%
X = t_pos(:,1);        
Y = t_pos(:,2);  
vX = vel.x;   
vY = vel.y;

X1 = X_f1(:,1)+P1(1);
Y1 = X_f1(:,3)+P1(2);
vX1 = X_f1(:,2);
vY1 = X_f1(:,4);

X2 = X_f2(:,1)+P2(1);
Y2 = X_f2(:,3)+P2(2);
vX2 = X_f2(:,2);
vY2 = X_f2(:,4);

X3 = X_fCC(:,1);
Y3 = X_fCC(:,3);
vX3 = X_fCC(:,2);
vY3 = X_fCC(:,4);

figure;
hold on; grid on; 
plot(abs(X1-X),'LineWidth',1); 
plot(abs(X2-X),'LineWidth',1); 
plot(abs(X3-X),'LineWidth',1); 
str1 = ['X1 RMSE:',num2str(sqrt(mean((X1-X).^2)))];
str2 = ['X2 RMSE:',num2str(sqrt(mean((X2-X).^2)))];
str3 = ['X3 RMSE:',num2str(sqrt(mean((X3-X).^2)))];
legend(str1,str2,str3); axis tight;

figure;
hold on; grid on; 
plot(abs(Y1-Y),'LineWidth',1); 
plot(abs(Y2-Y),'LineWidth',1); 
plot(abs(Y3-Y),'LineWidth',1); 
str1 = ['Y1 RMSE:',num2str(sqrt(mean((Y1-Y).^2)))];
str2 = ['Y2 RMSE:',num2str(sqrt(mean((Y2-Y).^2)))];
str3 = ['Y3 RMSE:',num2str(sqrt(mean((Y3-Y).^2)))];
legend(str1,str2,str3); axis tight;

figure;
hold on; grid on; 
plot(abs(vX1-vX),'LineWidth',1); 
plot(abs(vX2-vX),'LineWidth',1); 
plot(abs(vX3-vX),'LineWidth',1); 
str1 = ['vX1 RMSE:',num2str(sqrt(mean((vX1-vX).^2)))];
str2 = ['vX2 RMSE:',num2str(sqrt(mean((vX2-vX).^2)))];
str3 = ['vX3 RMSE:',num2str(sqrt(mean((vX3-vX).^2)))];
legend(str1,str2,str3); axis tight;

figure;
hold on; grid on; 
plot(abs(vY1-vY),'LineWidth',1); 
plot(abs(vY2-vY),'LineWidth',1); 
plot(abs(vY3-vY),'LineWidth',1); 
str1 = ['vY1 RMSE:',num2str(sqrt(mean((vY1-vY).^2)))];
str2 = ['vY2 RMSE:',num2str(sqrt(mean((vY2-vY).^2)))];
str3 = ['vY3 RMSE:',num2str(sqrt(mean((vY3-vY).^2)))];
legend(str1,str2,str3); axis tight;