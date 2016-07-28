function y = approximate(x, fit)

y = 0;
degree = length(fit.coeff);
for i=1:degree
    y = y + fit.coeff(i)*x^(degree-i);
end