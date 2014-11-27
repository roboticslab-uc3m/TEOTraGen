function test_DH_parameters(A, q)
%  This function is used to check the matrix A generated through
%  Denavit-Hartenberg Parametersn = length(q);
[a,b,n] = size(A);
for jj=1:n
    q(jj)
    0

    test=subs(A(:,:,jj), q(jj), 0);
    p = test(1:3,4)
    R = test(1:3,1:3)
    pause
    
    pi/2
    test=subs(A(:,:,jj), q(jj), sym(pi/2));
    p = test(1:3,4)
    R = test(1:3,1:3)
    pause
end

end

