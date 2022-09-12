function [P,X] = BC_Fusion(X1,P1,K1,X2,P2,K2,H,P,F)
P1 = diag(P1);
P1 = diag(abs(P1));
P2 = diag(P2);
P2 = diag(abs(P2));
P = (eye(4) - K2*H)*(F*P*F.')*(eye(4) - K1*H).';
X = X2 + (P2-P)*(P1+P2-2*P)^-1*(X1-X2);
end