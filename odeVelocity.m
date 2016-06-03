tspan=[0 2];
y0=2;
[t,y]=ode15s(@(t,y)-10*t, tspan, y0);
plot(t,y,'-o')
