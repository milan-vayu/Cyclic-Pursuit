%code for formation around target with desired radius and inclination
n=input('number of agnets');                          % take integer n as input
k_th=0.1; k_r=0.3; k_phi=0.5;                         %controller gains taken arbitrarily
x=input('enter initial x cordinates of each agent');  %input a row of initial x coordinates of all agents
y=input('enter initial y coordinates of each agent'); %input a row of initial y coordinates of all agents
z=input('enter initial z coordinates of each agent'); %input a row of initial z coordinates of all agents

%% to input x,y,z coordinates use this code >> randi(number of your wish,1,n)

%%
psi=input('desired elevation angle');                   
R=input('desired radius');
[theta, phi , r ]= cart2sph(x,y,z);                   %[azimuth, elevation, r]

%preallocation of variables used
store_r=ones(n,50000);                            %to store r of each agents at different time steps(1 ms)
store_theta=ones(n,50000);                        %to store theta of each agents at different time steps(1 ms)
store_phi=ones(n,50000);                          %to store phi of each agents at different time steps(1 ms)
theta_dot=zeros(1,n);                     %row of theta_dot of all agents at any instant of time
phi_dot=zeros(1,n);                       %row of phi_dot of all agents at any instant of time
r_dot=zeros(1,n);                         %row of r_dot of all agents at any instant of time
for t=1:1:50000                      %each loop updates r,theta and phi of each agent
    for i=1:n
        %control law used
        theta_dot(i)=k_th*(theta(i)-theta(mod(i,n)+1));
        phi_dot(i)=k_phi*(psi-phi(i));
        r_dot(i)=k_r*(R-r(i));
    end
    %for finding updated theta, phi and r 
    %discrete integration  %% time step=0.001 s
    theta=theta_dot*0.001 + theta;   
    phi=phi_dot*0.001 + phi;
    r=r_dot*0.001 + r;
 
    for i=1:n                               %to store r,theta and phi of each agent at each time step
        store_r(i,t)=r(i);
        store_theta(i,t)=theta(i);
        store_phi(i,t)=phi(i);
    end
end
%for plotting
hold on;
grid on;
for i=1:n
    [xa, ya, za]=sph2cart(store_theta(i,:),store_phi(i,:),store_r(i,:));
    plot3(xa,ya,za);
    plot3(xa(1),ya(1),za(1), 'bs');               %for marking initial point
    plot3(xa(50000),ya(50000),za(50000), 'ro');   %for marking final point
end 
%for marking target coordinates which is (0 , 0, 0)
plot3(0,0,0,'c+');
%for annotation of plot
dim = [.2 .5 .3 .3];
str = 'square & circle corresponds to initial and final point respectively';
annotation('textbox',dim,'String',str,'FitBoxToText','on');
%labelling of axes
xlabel('X');ylabel('Y');zlabel('Z');
