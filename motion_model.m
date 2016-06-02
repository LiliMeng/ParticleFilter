%use the polar coordiante
%Author: Lili Meng

for i=1:100
  rho(i)=normrnd(1,0.1);
  theta(i)=normrnd(1,0);
end

[x1,y1] = pol2cart(theta, rho);

figure
%polar(theta, rho, '*');
plot(x1,y1,'*')
