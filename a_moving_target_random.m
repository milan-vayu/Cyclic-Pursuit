% this code takes random initial positions
n=input('Enter the number of agents:');
max_value=input('enter the maximum value that random number generator will give as agents position: ');
store = zeros(n,3,100001); %to store the coordinates of agents. the 3rd index is time 
store(:,:,1)=(rand(n,3)-0.5)*2*max_value; %random initial positions of agents
k = 0.3; %gain
d = 0.001; %small time interval for which velocity is considered constant to calculate next position depending on the current position
alpha=1.5*pi/n; %offset to bearing angle(bearing angle = angle of line connecting to agents)
T=[cos(alpha) sin(alpha) 0;-sin(alpha) cos(alpha) 0;0 0 1]; % offset matrix
kt=2*sin(pi/n)*sin(alpha-pi/n); %gain
zt=(rand(1,3)-0.5)*2*max_value; %random target
vx=0;vy=0;vz=0; %here we can give any velocity to the target

%following is a loop to store positions of all agents
for i = 1:100000
    for j = 1:n
        u=k*T*(store(mod(j,n)+1,:,i)'-store(j,:,i)')-kt*k*(store(j,:,i)'-zt'); %velocity of 'j'th agent as given in the document
        store(j,:,i+1)= store(j,:,i)+ u'.*d; %from the velocity we can get next position by assuming constant velocity for this small time interval
        zt(1)=zt(1)+d*vx; %from the given velocity of target we calculate next position of target
        zt(2)=zt(2)+d*vy;
        zt(3)=zt(3)+d*vz;
    end
end

%for plotting
for i=1:n
    
    for k=1:100001
        x1(k)=store(i,1,k);
        y1(k)=store(i,2,k);
        z1(k)=store(i,3,k);
    end
    plot3(x1(1),y1(1),z1(1),'bs'); %initial position of agents is denoted by squares
    hold on
    plot3(x1(100001),y1(100001),z1(100001),'bo') % final position of agents is denoted by circles
    plot3(x1,y1,z1); %trajectory of agents
end
plot3(zt(1),zt(2),zt(3),'r+') %target position denoted by plus sign
grid on
xlabel('x'); ylabel('y'); zlabel('z');