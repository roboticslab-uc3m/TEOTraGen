function H = evaluate_analytical_hessian (p, q)
m = size(p,1);
n = size(q,1);
%[m, n] = size(J);
H = sym(zeros(m, n, n));
for jj = 1:m
    for kk = 1:n
        for ll = 1:n
            H(jj, kk, ll) = diff(diff(p(jj), q(ll)), q(kk));
        end
    end
end

% function H = hessian(f,varlist)
% %hessian(f)
% %Finds the Hessian matrix symbolically for symbolic input function f.
% %To use it, type syms <variables>.
% %for example:
% %syms x y
% %f = 3*x*y^2
% %varlist is the list of variables in f.
% %hessian(f,[x y])
% %ans =
% %[   0, 6*y]
% %[ 6*y, 6*x]
% %(c) Brad Ridder 2007. Feel free to use this under the GPL guidelines. If
% %you wish to add to this program, just leave my name and add yours to it.
% n = numel(varlist);
% hess = vpa(ones(n,n));
% for j = 1:n;
%     for i = 1:n;
%         hess(j,i) = diff(diff(f,varlist(i),1),varlist(j),1);
%     end
% end
% H=hess;