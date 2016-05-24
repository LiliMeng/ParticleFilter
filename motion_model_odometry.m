function  [new_position] = motion_model_odometry( odo_reading, initial_position )
 
%UNTITLED4 Summary of this function goes here
 
%   Detailed explanation goes here
 

a1=0.01;
a2=0.01;
a3=0;
a4=0;

xb=odo_reading(1,1);
yb=odo_reading(1,2);
thb=odo_reading(1,3);

xbp=odo_reading(2,1);
ybp=odo_reading(2,2);
thbp=odo_reading(2,3);

%odo_reading(1,:)=[xb yb thb];
 
%odo_reading(2,:)=[xbp ybp thbp];
 
 
 
dr1= atan2(ybp-yb,xbp-xb)-thb;
 
dt= sqrt((xb-xbp)^2+(yb-ybp)^2);
 
dr2= thbp-thb-dr1;
 
 
 
dr1b= dr1-normrnd(0, a1*dr1^2+a2*dt^2);
 
dtb= dt-normrnd(0, a3*dt^2+a4*dr1^2+a4*dr2^2);
 
dr2b= dr2-normrnd(0, a1*dr2^2+a2*dt^2);
 
 
 
xp= initial_position(1)+dtb*cos(initial_position(3)+dr1b);
 
yp= initial_position(2)+dtb*sin(initial_position(3)+dr1b);
 
thp= initial_position(3)+dr1b+dr2b;
 
new_position= transpose([xp;yp;thp])
 
end
