function H = differentiate_jacobian (J, q)
[m, n] = size(J);
H = sym(zeros(m, n, n));
for jj=1:n
    H(:,:,jj) = jacobian(J(:,jj), q);
end