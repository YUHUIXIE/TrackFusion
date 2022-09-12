function X_fCC = CC_EKF(r_mea1,a_mea1,r_mea2,a_mea2,Sigma,Sensor)
F = [1 1 0 0;
     0 1 0 0;
     0 0 1 1;
     0 0 0 1];
Q = [1/4*Sigma.u^2  1/2*Sigma.u^2             0              0;
     1/2*Sigma.u^2      Sigma.u^2             0              0;
                 0              0  1/4*Sigma.u^2  1/2*Sigma.u^2;
                 0              0  1/2*Sigma.u^2      Sigma.u^2];
P1 = Sensor.P1;
P2 = Sensor.P2;
X1 = [r_mea1(1)*cos(a_mea1(1)) + P1(1);0;r_mea1(1)*sin(a_mea1(1))+P1(2);0];
X2 = [r_mea2(1)*cos(a_mea2(1)) + P2(1);0;r_mea2(1)*sin(a_mea2(1))+P2(2);0];
X = (X1+X2)/2;
X1 = X;
X2 = X;
H = [1 0 0 0;
     0 0 1 0];
R0 = [Sigma.r^2          0;
           0    Sigma.a^2];
% P_1 = [Sigma.r^2    Sigma.r^2 0 0;
%        Sigma.r^2  2*Sigma.r^2 0 0;
%        0 0 Sigma.r^2    Sigma.r^2;
%        0 0 Sigma.r^2  2*Sigma.r^2];
% P_2 = P_1;
P_1 = eye(4);
P_2 = P_1;
N = length(r_mea1);
% X_1 = zeros(N,4);
% CellP1 = zeros(N,4,4);
% X_2 = zeros(N,4);
% CellP2 = zeros(N,4,4);
X_fCC = zeros(N,4);
for n = 1:N
    X_pre1 = F*X1;
    P_pre1 = F*P_1*F.' + Q;
    
    W1 = [cos(a_mea1(n)),-r_mea1(n)*sin(a_mea1(n));
          sin(a_mea1(n)), r_mea1(n)*cos(a_mea1(n))];
    R1 = W1*R0*W1.';
    K1 = P_pre1*H.'*(H*P_1*H.'+R1)^-1;
    X1 = X_pre1 + ...
         K1*([r_mea1(n)*cos(a_mea1(n))+P1(1);r_mea1(n)*sin(a_mea1(n))+P1(2)] - H*X_pre1);
    P_1 = (eye(4) - K1*H)*P_pre1;
    
 
    X_pre2 = F*X2;
    P_pre2 = F*P_2*F.' + Q;
    W2 = [cos(a_mea2(n)),-r_mea2(n)*sin(a_mea2(n));
          sin(a_mea2(n)), r_mea2(n)*cos(a_mea2(n))];
    R2 = W2*R0*W2.';
    K2 = P_pre2*H.'*(H*P_2*H.'+R2)^-1;
    X2 = X_pre2 + ...
         K2*([r_mea2(n)*cos(a_mea2(n))+P2(1);r_mea2(n)*sin(a_mea2(n))+P2(2)] - H*X_pre2);
    P_2 = (eye(4) - K2*H)*P_pre2;
       
    
%     P = (P_1^-1+P_2^-1)^-1;
%     X = P*(P_1^-1*X1+P_2^-1*X2);
    X = CC_Fusion(X1,P_1,X2,P_2);
    X = X.';
    X1 = X;
    X2 = X;
    X_fCC(n,:) = X;

end
end