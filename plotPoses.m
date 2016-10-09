pose=load('/home/lci/workspace/odometry/data_nn_pwc/output_motion/pose.txt');


x=pose(:,1);
y=pose(:,2);

figure

%plot(x,y,'*')

%hold on;
x_avg=mean(x)
y_avg=mean(y)

%plot(x_avg,y_avg,'o')

%hold on;


[x1,y1] = pol2cart(x,y);
k = convhull(x1,y1);
plot(x1(k),y1(k),'r-',x1,y1,'b*')

hold on;


theta = pose(:,3);
r = 0.0001; % magnitude (length) of arrow to plot
u = r * cos(theta); % convert polar (theta,r) to cartesian
v = r * sin(theta);
h = quiver(x1,y1,u,v);
