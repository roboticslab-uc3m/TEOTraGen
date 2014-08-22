function A = generate_symbolic_vector(name, n)
% This function creates a vector of n symbolic variables 'name'
A = sym(zeros(n, 1));
if n < 10
    for jj = 1:n
        A(jj) = sym([name, num2str(jj, '%d')], 'real');
    end
    return    
end
for jj = 1:9
	A(jj) = sym([name, num2str(jj, '0%d')], 'real');
end
for jj = 10:n
	A(jj) = sym([name, num2str(jj, '%d')], 'real');
end
end