function X_f = EKF(r_mea,a_mea,Sigma)
F = [1 1 0 0;
     0 1 0 0;
     0 0 1 1;
     0 0 0 1];
Q = [1/4*Sigma.u^2  1/2*Sigma.u^2             0              0;
     1/2*Sigma.u^2      Sigma.u^2             0              0;
                 0              0  1/4*Sigma.u^2  1/2*Sigma.u^2;
                 0              0  1/2*Sigma.u^2      Sigma.u^2];
X = [r_mea(1)*cos(a_mea(1));0;r_mea(1)*sin(a_mea(1));0];
H = [1 0 0 0;
     0 0 1 0];
R0 = [Sigma.r^2          0;
           0    Sigma.a^2];
P = eye(4);
N = length(r_mea);
X_f = zeros(N,4);
for n = 1:N
    X_pre = F*X;
    P_pre = F*P*F.' + Q;
    
    W = [cos(a_mea(n)),-r_mea(n)*sin(a_mea(n));
         sin(a_mea(n)), r_mea(n)*cos(a_mea(n))];
    R = W*R0*W.';
    K = P_pre*H.'*(H*P*H.'+R)^-1;
    X = X_pre + ...
        K*([r_mea(n)*cos(a_mea(n));r_mea(n)*sin(a_mea(n))] - H*X_pre);
    P = (eye(4) - K*H)*P_pre;
    
    X_f(n,:) = X;
end
end