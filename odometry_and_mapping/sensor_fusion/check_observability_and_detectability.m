% Check observability
n = length(F(:,:,i-1));
O = [];
for j=1:n
   O = [O; H*F(:,:,i-1).^(j-1)];
end
if rank(O)==n
   disp('------------------------- Observable!!!!!-------------------')
end

% Check detectability
lambdas = eig(F(:,:,i-1));
lambdas = lambdas(abs(lambdas)>=1);
detectable = true;
for k=1:length(lambdas)
   D = [F(:,:,i-1)' - lambdas(k)*eye(n), H'];
   if rank(D)~=n
    detectable = false;
    break;
   end
end
if detectable
   disp('---------------------- Detectable!!! -----------------------')
end