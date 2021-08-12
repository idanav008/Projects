clear all
close all
clc

global Rho Nhi S_Bullet S_Mesh D_Mesh D_Bullet L_Bullet m g  T_Movement_Drone dt 
%global opz_aereo_y

Rho=1.2; %Air Density [kg/m3]
S_Bullet=0.005153; %Surface First stage [m2]
S_Mesh=0.08; %Surface second stage
D_Mesh=0.01;
Lc_Mesh=D_Mesh;
Cd_Mesh=0; %Drag coefficient Second stage
m=4; %mass [kg]
Nhi=1.5*10^-5;%Air viscosity
D_Bullet=0.081;%Bullet diameter [m]
L_Bullet=0.5;%Bullet length [m]
g=9.81; %Gravitational Acceleration [m/s2]
tf=4; %Simulation final time [s]
T_Movement_Drone=4; %time until interception
dt=0.1;
T_Bullet=0;
hitpos=0; 
Hit_flag=0;
x0_mesh=0;
y0_mesh=0;
Cd_Bullet=0.44*(D_Bullet/L_Bullet)+4*0.004*(L_Bullet/D_Bullet)+4*0.004*(D_Bullet/L_Bullet)^0.5; %Drag coefficient First stage [-]
Vterminal_Bullet=sqrt((2*m*g)/(Cd_Bullet*Rho*S_Bullet));
Vterminal_Mesh=sqrt((2*m*g)/(Cd_Mesh*Rho*S_Mesh));
V0y_Bullet=0;
V0x_Bullet=0;
Vy_Bullet=[0 0 0 0 0 0];
Vx_Bullet=[0 0 0 0 0 0];
Output_Vector=[0 0 0 0 0]; %save results by: velocity,angle,y coordinate,x coordinate
Output_Velocity=0;

%Initial conditions
x0=0; %Initial x [m]
y0=0; %Initial y [m]

     
        %input-
xi_drone= input('Enter initial x position in meters: ');%x initial target
yi_drone= input('Enter initial y position in meters: ');%y initial target
zi_drone= input('Enter initial z position in meters: ');%z initial target
vx_drone=input('Enter initial vx position in meters: ');%vx initial target 
vy_drone=input('Enter initial vy position in meters: ');%vy initial target 
vz_drone=input('Enter initial vz position in meters: ');%vz initial target 

%{        
%System configuration
figure(1)
clf
subplot(2,2,[1 3])
hold on
grid on
xlabel('range [m]','FontWeight','bold')
ylabel('Height [m]','FontWeight','bold')
set(gca,'FontWeight','bold')
%} 

%Drone coordinates after movment
xdrone=xi_drone+ vx_drone*T_Movement_Drone;
ydrone=yi_drone+ vy_drone*T_Movement_Drone;
zdrone=zi_drone+ vz_drone*T_Movement_Drone;

%Rotation angle
Output_Vector(5)= rad2deg(atan(zdrone/xdrone));

%{
%experiment conditions
xdrone=100;
ydrone=100;
zdrone=100;
vx_drone=0;
vy_drone=0;
vz_drone=0;
%}
        
alpha_ini=10:1:80; %Launch angle
V0=[73,110,137,162,195,224];%Launch speeds [m/s]
alpha0=deg2rad(alpha_ini);%Convert degrees to Rad
%alpha0=deg2rad(80);
%V0x_Bullet=V0*cos(alpha0);%initial speed x
%V0y_Bullet=V0*sin(alpha0);%initial speed y
x=x0;
y=y0;
        
for i_velocity=1:length(V0)    %loop for veloity pick
          
    for i_angle=1:length(alpha0)  %loop for angle pick
            
        T_Bullet=0;
        V0x_Bullet=V0(i_velocity)*cos(alpha0(i_angle));%initial speed x
        V0y_Bullet=V0(i_velocity)*sin(alpha0(i_angle));%initial speed y
        x=x0;
        y=y0;
        Data=zeros(1000,3);
            
        %Bullet movement calculation
        while T_Bullet<0.5
                
            Vy_Bullet(i_velocity)= Vterminal_Bullet*((V0y_Bullet-Vterminal_Bullet*tan((g*dt)/Vterminal_Bullet))/(Vterminal_Bullet+V0y_Bullet*tan((g*dt)/Vterminal_Bullet)));
            Vx_Bullet(i_velocity)= (power(Vterminal_Bullet,2)*V0x_Bullet)/(power(Vterminal_Bullet,2)+g*dt*V0x_Bullet);
                
            y= y0+(power(Vterminal_Bullet,2)/(2*g))*(log((power(V0y_Bullet,2)+power(Vterminal_Bullet,2))/(power(Vy_Bullet(i_velocity),2)+power(Vterminal_Bullet,2))));
            x= x0+(power(Vterminal_Bullet,2)/(g))*(log((power(Vterminal_Bullet,2)+g*V0x_Bullet*dt)/power(Vterminal_Bullet,2)));
                
            %progression to the next calculation
            y0= y;
            x0= x;
            V0y_Bullet= Vy_Bullet(i_velocity);
            V0x_Bullet= Vx_Bullet(i_velocity);
            T_Bullet= T_Bullet+dt;
            
        end
            
        
        %advance latest stats from bullet to mesh   
        V0y_Mesh=Vy_Bullet(i_velocity);
        V0x_Mesh=Vx_Bullet(i_velocity);
        T_Mesh=T_Bullet;
            
        Max_Flag=0;
           
        Ymax=y+(power(Vterminal_Mesh,2)/(2*g))*log((power(V0y_Mesh,2)+power(Vterminal_Mesh,2))/(power(Vterminal_Mesh,2)));
            
        i_data=1;
       
        %mesh movement calculation
        while (T_Mesh<4) && (Max_Flag==0)
             
            V_Mesh= sqrt(power(V0y_Mesh,2)+power(V0x_Mesh,2));
            Re= (V_Mesh*Lc_Mesh)/Nhi;
            Cd_Mesh= 6.8/power(Re,0.89)+1.96/power(Re,0.5)-1/(1/(4*power(10,-4)*Re)+Re/(1.1*power(10,-3)))+1.18;
            Vterminal_Mesh=sqrt((2*m*g)/(Cd_Mesh*Rho*S_Mesh));
                  
                
            Vy_Mesh= Vterminal_Mesh*(V0y_Mesh-Vterminal_Mesh*tan((g*dt)/Vterminal_Mesh))/(Vterminal_Mesh+V0y_Mesh*tan((g*dt)/Vterminal_Mesh));
            Vx_Mesh= (power(Vterminal_Mesh,2)*V0x_Mesh)/(power(Vterminal_Mesh,2)+g*dt*V0x_Mesh);
                
            y= y0+(power(Vterminal_Mesh,2)/(2*g))*log((power(V0y_Mesh,2)+power(Vterminal_Mesh,2))/(power(Vy_Mesh,2)+power(Vterminal_Mesh,2)));
            x= x0+(power(Vterminal_Mesh,2)/g)*log((power(Vterminal_Mesh,2)+g*V0x_Mesh*dt)/power(Vterminal_Mesh,2));
                
            Data(i_data,1)= T_Mesh;
            Data(i_data,2)= y;
            Data(i_data,3)= x;
            i_data= i_data+1;
                
            Ymax=y+(power(Vterminal_Mesh,2)/(2*g))*log((power(V0y_Mesh,2)+power(Vterminal_Mesh,2))/(power(Vterminal_Mesh,2)));
                
            if Ymax-y<=0.0001 || Vy_Mesh<=0 || Vx_Mesh<=0
                Max_Flag=1;
            end
            
            %progression to the next calculation
            y0= y;
            x0= x;
            V0y_Mesh= Vy_Mesh;
            V0x_Mesh= Vx_Mesh;
            T_Mesh= T_Mesh+dt;
                
        end
            
            hitpos= find(abs(Data(:,2)-ydrone)<=2);
            if hitpos
                hit= zeros(length(hitpos),3);
                for i_hit=1:length(hitpos)
                    hit(i_hit,1)= Data(hitpos(i_hit,1),1);
                    hit(i_hit,2)= Data(hitpos(i_hit,1),2);
                    hit(i_hit,3)= Data(hitpos(i_hit,1),3);
                end
                
                for j_hit=1:length(hitpos)
                    if (abs(hit(j_hit,3)-xdrone)<=2)
                        Hit_flag=1;
                        Output_Vector(3)=hit(j_hit,2);
                        Output_Vector(4)=hit(j_hit,3);
                        Output_Vector(2)= i_angle+9;
                        
                        if i_velocity==1
                            Output_Velocity=73;                       
                        
                        elseif i_velocity==2
                            Output_Velocity=110;
                        
                        elseif i_velocity==3
                            Output_Velocity=137;
                        
                        elseif i_velocity==4
                            Output_Velocity=162;
        
                        elseif i_velocity==5
                            Output_Velocity=195;
        
                        elseif i_velocity==6
                            Output_Velocity=224;
        
                        end
        
                        Output_Vector(1)=Output_Velocity;
                        
                        break;
                    end
                    if Hit_flag==1
                        break;
                    end
                end
            end
            
            if Hit_flag==1
                break;
            end
            
            y0= 0;
            x0= 0;
            V0y_Bullet= 0;
            V0x_Bullet= 0;
            T_Bullet= 0;
            V0y_Mesh= 0;
            V0x_Mesh= 0;
            T_Mesh= 0;
            
    end
        
    if Hit_flag==1
        break;
    end
    
    
    if Hit_flag==1
        break;
    end
    
    i_angle=0; 
          
end

if Hit_flag==1 && abs(Output_Vector(5))<=135
    fprintf('Launch angle is %d [deg]' ,Output_Vector(2));
    fprintf('\n');
    fprintf('Launch velocity is %d [m/s]' ,Output_Vector(1));
    fprintf('\n');
    fprintf('Intersection point: (%4.2f,%4.2f) [m]' ,Output_Vector(4),Output_Vector(3));
    fprintf('\n');
    fprintf('Rotational angle of launch is %4.2f' ,Output_Vector(5));
    fprintf('\n');
else
    disp('The targent is not in range')
end


      