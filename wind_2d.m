%code considering wind but not the aerodynamic drag
n=input('number of agents');             % take n as input
al=pi/n;                                 %initial offset which is constant throughout
k=0.3;                                   %controller gain
T=[cos(al),sin(al);-sin(al),cos(al)];    %rotation matrix
v_wind=input('input velocity of wind in 2d');  %input wind velocity vector in 2d i.e., [1;5]
x(1:n)=input('initial x coordinates');   %input row of x coordinates of n agents
y(1:n)=input('initial y coordinates');   %input row of y coordinates of n agents
%% to input x,y coordinates  use this code >> randi(number of your wish,1,n)
%%
z=zeros(2,n);                            %preallocation (for shorter run time)
                                         %note z here is matrix for storing
                                         %x and y coordinates of each agent
for i=1:n 
    z(1:2,i)=[x(i);y(i)];                %each column of matrix z contains  x and y coordinates of i th agent 
end
store_position=ones(2,100000,n);         %preallocation of 3d array which stores x,y coordinates at each  
%instant of time step form 1 to 100000. And each page of 3d array contains information
%about i th agents
for t=1:1:100000
    for i=1:n
        u(1:2,i)=(k*T*(z(1:2,mod(i,n)+1)-z(1:2,i)))-v_wind; %updating velcity of i th agent after each time step(1 ms)
    end
    for i=1:n
        z(1:2,i)=u(1:2,i).*0.001 + z(1:2,i);      % updating position of i th agent after each time step(1 ms)
        store_position(1:2,t,i)=z(1:2,i);         
    end
end
%for plotting trajectory of each agents.
grid on;
hold on;
for i=1:n
    plot(store_position(1,:,i),store_position(2,:,i));
    plot(store_position(1,1,i),store_position(2,1,i), 'rs');
    plot(store_position(1,100000,i),store_position(2,100000,i) , 'bo');
    hold on;
end
%for annotation of plot
dim = [.2 .5 .3 .3];
str = 'square & circle corresponds to initial and final point respectively';
annotation('textbox',dim,'String',str,'FitBoxToText','on'); 
%labelling of axes
xlabel('X');ylabel('Y');