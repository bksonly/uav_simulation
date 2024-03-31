function phi = ln_matrix(R)
phi=zeros([3,3]);
for n=0:100000
    phi=phi+(-1)^n/(n+1)*(R-eye(3))^(n+1);
end

end