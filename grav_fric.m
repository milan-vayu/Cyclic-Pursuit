n=input('Enter the number of agents:');
store = zeros(n,2,100001); %to store the coordinates of agents. the 3rd index is time
store(:,:,1)=input('enter initial coordinates: '); %initial position of agents
theta=input('enter the tilt angel about y axis: ');
m=1; %mass of agents
mu=0.2; %kinetic friction coefficiet
g=9.8; %gravitational cons.
k = 0.3; %gain
d = 0.001; %small time interval for which velocity is considered constant to calculate next position depending on the current position
alpha=pi/n; %offset to bearing angle(bearing angle = angle of line connecting to agents)
T=[cos(alpha) sin(alpha);-sin(alpha) cos(alpha)];% offset matrix
v=zeros(n,2); %to store the velocity of agents(initially zero)

%following is a loop to store positions of all agents
for i = 1:100000
    for j = 1:n
        v_req=k*T*(store(mod(j,n)+1,:,i)-store(j,:,i))';
        delta_v=(v_req'-v(j,:));
        a_thrust=(delta_v/norm(delta_v)).*20;
        if norm(v(j,:))~=0
            a=[-m*g*sin(theta) 0] - (v(j,:)/norm(v(j,:))).*mu*m*g*cos(theta)+a_thrust;
        else
            a=[-m*g*sin(theta) 0]+a_thrust;
        end
        v(j,:)=v(j,:)+a.*d; %velocity of 'j'th agent
        store(j,:,i+1)= store(j,:,i)+ v(j,:).*d; %from the velocity we can get next position by assuming constant velocity for this small time interval
    end
end

%for plotting
for i=1:n
    for k=1:100001
        p=store(i,:,k)';
        x(k)=p(1);
        y(k)=p(2);
    end
    plot(x(1),y(1),'bs'); %initial position of agents is denoted by squares
    hold on
    plot(x(100001),y(100001),'bo') % final position of agents is denoted by circles
    plot(x,y)  %trajectory of agents
end
grid on
xlabel('x'); ylabel('y');