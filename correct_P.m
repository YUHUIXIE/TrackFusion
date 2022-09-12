function P = correct_P(P)
N = size(P);
for n = 1:N
    if P(n,n) < 0
        P(n,n) = P(n,n)*-1;
    end
end
end