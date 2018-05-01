e = -1:0.0001:1;
alpha = 0.5;
deta = 0.01;
y = abs(e).^alpha.* sign(e);

plot(e,y)
