clear all
close all 

xb=0.01;
yb=0.01;
thb=0.01;
 
initial_position=transpose([0,0,0]);
 
xbp=4;
ybp=3;
thbp=0.6454;
 

 
odo_reading(1,:)=[xb yb thb];
 
odo_reading(2,:)=[xbp ybp thbp];



for i=1:500
new_P(i,:)= motion_model_odometry( odo_reading, initial_position);
end 


figure
plot(new_P(:,1),new_P(:,2),'*')
