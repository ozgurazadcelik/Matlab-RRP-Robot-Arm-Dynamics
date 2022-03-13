function robot_param=dynNS_Energy(u)
global h1 d2 l3 m1 m2 m3 Ixx2 Ixx3 Iyy2 Iyy3 Izz1 Izz2 Izz3 g;

q1=u(1); %position of first joint
q2=u(4); %position of 2nd joint
q3=u(7); %positionof 3rd joint
dq1=u(2); %velocity of first joint
dq2=u(5); %velocity of 2nd joint
dq3=u(8); %velocity of 3rd joint
ddq1=u(3); %acceleration of first joint
ddq2=u(6); %acceleration of 2nd joint
ddq3=u(9); %acceleration of 3rd joint

s1=sin(q1*pi/180); 
s2=sin(q2*pi/180); 
s3=sin(q3*pi/180); 

c1=cos(q1*pi/180);
c2=cos(q2*pi/180);
c3=cos(q3*pi/180);


%%% Mass Matrix

M11=Izz1+(1/4)*m2*d2^2+s2^2*(Ixx2+Ixx3+m3*(0.25*l3^2-l3*q3+q3^2))+c2^2*(Iyy2+Izz3)+m3*d2^2;
M12=d2*m3*c2*(0.5*l3-q3);
M13=-d2*m3*s2;
M21=d2*m3*c2*(0.5*l3-q3);
M22=Izz2+Iyy3+m3*(0.25*l3^2-l3*q3+q3^2);
M23=0;
M31=-d2*m3*s2;
M32=0;
M33=m3;

M=[M11 M12 M13; M21 M22 M23; M31 M32 M33];

robot_param=[M];

%%% Coriolis and Centrifugal Forces

C1= 2*[s2*c2*(Ixx2+Ixx3-Iyy2-Izz3+m3*(0.25*l3^2-l3*q3+q3^2))]*dq1*dq2-[d2*m3*s2*(0.5*l3-q3)]*dq2^2-2*[d2*m3*c2]*dq2*dq3+[m3*s2^2*(-l3+2*q3)]*dq1*dq3;
C2=-s2*c2*(Ixx2+Ixx3-Iyy2-Izz3+m3*(0.25-l3^2-l3*q3+q3^2))*dq1^2+m3*(-l3+2*q3)*dq2*dq3;
C3=m3*s2*(0.5*l3-q3)*dq1^2+m3*(0.5*l3-q3)*dq2^2;
C=[C1;C2;C3];

%%% Gravity Vector

G1=0; 
G2=m3*g*s2*(0.5*l3-q3);
G3=m3*g*c2;
G=[G1;G2;G3];

%%% Joint Torques

ddq=[ddq1;ddq2;ddq3]; 
torque= M*ddq+C+G;

robot_param=[C G torque];

%%% Kinetic Energy

K1= 0.5*Izz1*dq1^2
K2=0.5*(0.25*m2*d2^2+s2^2*Ixx2+c2^2+Iyy2)*dq1^2+0.5*Izz2*dq2^2;
K3=0.5*[s2^2*(Ixx3+m3*(0.25*l3^2-l3*q3+q3^2))+c2^2*Izz3+m3*d2^2]*dq1^2+0.5*[m3*(0.25*l3^2-l3*q3+q3^2)+Iyy3]*dq2^2+(0.5*m3*dq3^2)+[d2*m3*c2*(0.5*l3-q3)]*dq1*dq2-[d2*m3*s2]*dq1*dq3;

KinEn=[K1;K2;K3];

%%% Potential Energy

P1=m1*g*(h1/2);
P2=m2*g*h1;
P3=m3*g*[c2*(-0.5*l3+q3)+h1];
PotEn=[P1;P2;P3];

robot_param=[KinEn;PotEn];
