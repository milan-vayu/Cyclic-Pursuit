%this code not giving corerct radius
%also the agent inside the circle remain inside
target=input('enter target location: ');
radius=input('enter radius of circle: ');
n=input('enter number of agents: ');
store=zeros(n,2,1000001);
store(:,:,1)=input('enter initial coordinates: '); %initial position of agents
x=[-pi/2 0 pi];
s=10;       %constant velocity
k=1.3;
alpha=10;
t=0.001;

for j=1:n
    gama_track(j)=atan2(-target(2)+store(j,2,1),-target(1)+store(j,1,1));
end

for j=1:n
    count(j)=0;
end



for i=1:1000000
    for j=1:n
        d=sqrt((target(1)-store(j,1,i))^2+(target(2)-store(j,2,i))^2);
        
        gama1=atan2(-target(2)+store(j,2,i),-target(1)+store(j,1,i));
        if gama1<0
            gama1=pi+(pi+gama1);
        end
        
        
      
        if gama_track(j)>1.7*pi & gama1<0.3*pi
            count(j)=count(j)+1;
        end
        if gama_track(j)<0.3*pi & gama1>1.7*pi
            count(j)=count(j)-1;
        end
        
        gama=gama1+2*pi*count(j);
        
        
        if d>2*radius
            xd=gama-5*pi/6;
            xc=gama-5*pi/6+s/d*sin(x(j)-gama);
        else
            xd=gama-pi/2-pi/3*(abs(d-radius)/radius)^k;
            xc=xd-  (s/(alpha*d))*sin(x(j)-gama)-k*s*pi/(3*alpha*radius^k)*((abs(d-2*radius))^(k-1))*cos(x(j)-gama);
        end
        
        x_dot=alpha*(xc-x(j));
        x(j)=x(j)+x_dot*t;
        store(j,:,i+1)=store(j,:,i)+[s*cos(x(j)) s*sin(x(j))].*t;
        gama_track(j)=gama1;
    end
end

%for plotting
for i=1:n
    for k=1:1000001
        p=store(i,:,k)';
        x(k)=p(1);
        y(k)=p(2);
    end
    plot(x(1),y(1),'bs'); %initial position of agents is denoted by squares
    hold on
    plot(x(1000001),y(1000001),'bo') % final position of agents is denoted by circles
    plot(x,y)  %trajectory of agents
end
plot(target(1),target(2),'r+')
grid on
xlabel('x'); ylabel('y');
clear