function robot_param=dynNS(u)
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


%%% Mass Matrix %%

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



