%code for circle formation around desired target %%needs correction in algorithm
%each page contains coordinates of each agent at different times
n=input('number of agents');             % take n as input
al=0.5*pi/n;                             %initial offset
k=input('controller gain');              %controller gain
T=[cos(al),sin(al) 0;-sin(al),cos(al) 0;0 0 1];    %trasformation matrix
x(1:n)=input('initial x coordinates');
y(1:n)=input('initial y coordinates');
zz(1:n)=input('initial z coordinates');
zt=[0;0;1];         %coordinates of coordinates
kt=2*sin(pi/n)*sin(al-pi/n);
for i=1:n 
    z(1:3,i)=[x(i);y(i);zz(i)];
end
store_position=ones(3,100000,n);            %for preallocation purpose
for t=1:1:100000
    for i=1:n
        u(1:3,i)=(k*T*(z(1:3,mod(i,n)+1)-z(1:3,i))-kt*zt);
    end
    for i=1:n
        z(1:3,i)=u(1:3,i).*0.001 + z(1:3,i);
        store_position(1:3,t,i)=z(1:3,i);
    end
end
for i=1:n
    plot3(store_position(1,:,i),store_position(2,:,i),store_position(3,:,i));
    hold on;
end
        
    