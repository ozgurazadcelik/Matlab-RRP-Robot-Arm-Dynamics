load q1.mat; load  dq1.mat; load ddq1.mat; 
load q2.mat; load dq2.mat; load ddq2.mat;
load q3.mat; load dq3.mat; load ddq3.mat;


load M11.mat; load  M12.mat; load M13.mat; 
load M21.mat; load M22.mat; load M23.mat;
load M31.mat; load M32.mat; load M33.mat;

figure(1);
subplot(3,3,1);
plot(M11(1,:));
ylabel('M11');

subplot(3,3,2);
plot(M12(1,:));
ylabel('M12')

subplot(3,3,3);
plot(M13(1,:));
ylabel('M13')

subplot(3,3,4);
plot(M21(1,:));
ylabel('M21')

subplot(3,3,5);
plot(M22(1,:));
ylabel('M22')

subplot(3,3,6);
plot(M23(1,:));
ylabel('M23')

subplot(3,3,7);
plot(M31(1,:));
ylabel('M31')

subplot(3,3,8);
plot(M32(1,:));
ylabel('M32')

subplot(3,3,9);
plot(M33(1,:));
ylabel('M33')


hold on
figure(2);
subplot(3,3,1);
plot(q1(1,:));
title('q1');
ylabel('Position')

subplot(3,3,2);
plot(dq1(1,:));
title('dq1')
ylabel('Velocity')

subplot(3,3,3);
plot(ddq1(1,:));
title('ddq1')
ylabel('Acceleration')

subplot(3,3,4);
plot(q2(1,:));
title('q2')
ylabel('Position')

subplot(3,3,5);
plot(dq2(1,:));
title('dq2')
ylabel('Velocity')

subplot(3,3,6);
plot(ddq2(1,:));
title('ddq2')
ylabel('Acceleration')

subplot(3,3,7);
plot(q3(1,:));
title('q3')
ylabel('Position')

subplot(3,3,8);
plot(dq3(1,:));
title('dq3')
ylabel('Velocity')

subplot(3,3,9);
plot(ddq3(1,:));
title('ddq3')
ylabel('Acceleration')

%%%

load C1.mat; load  C2.mat; load C3.mat; 
load G1.mat; load  G2.mat; load G3.mat; 
load CGTorque1.mat; load  CGTorque2.mat; load CGTorque3.mat; 

figure(3);
subplot(3,3,1);
plot(C1(1,:));
ylabel('C1');

subplot(3,3,2);
plot(C2(1,:));
ylabel('C2')

subplot(3,3,3);
plot(C3(1,:));
ylabel('C3')

subplot(3,3,4);
plot(G1(1,:));
ylabel('G1')

subplot(3,3,5);
plot(G2(1,:));
ylabel('G2')

subplot(3,3,6);
plot(G3(1,:));
ylabel('G3')

subplot(3,3,7);
plot(CGTorque1(1,:));
ylabel('CGTorque1')

subplot(3,3,8);
plot(CGTorque2(1,:));
ylabel('CGTorque2')

subplot(3,3,9);
plot(CGTorque3(1,:));
ylabel('CGTorque3')

hold on

%%%

load K1.mat; load  K2.mat; load K3.mat; 
load P1.mat; load  P2.mat; load P3.mat; 
load Lagrange.mat; load  Hamilton.mat;

figure(4);
subplot(3,3,1);
plot(K1(1,:));
title('K1');

subplot(3,3,2);
plot(K2(1,:));
title('K2');

subplot(3,3,3);
plot(K3(1,:));
title('K3');

subplot(3,3,4);
plot(P1(1,:));
title('P1');

subplot(3,3,5);
plot(P2(1,:));
title('P2');

subplot(3,3,6);
plot(P3(1,:));
title('P3');

subplot(3,3,7);
plot(Lagrange(1,:));
title('Lagrange');

subplot(3,3,8);
plot(Hamilton(1,:));
title('Hamilton');

hold on

%%%

load fx1.mat; load  fy1.mat; load fz1.mat; 
load nx1.mat; load  ny1.mat; load nz1.mat; 

figure(5);
subplot(2,3,1);
plot(fx1(1,:));
title('fx1');

subplot(2,3,2);
plot(fy1(1,:));
title('fy1');

subplot(2,3,3);
plot(fz1(1,:));
title('fz1');

subplot(2,3,4);
plot(nx1(1,:));
title('nx1');

subplot(2,3,5);
plot(ny1(1,:));
title('ny1');

subplot(2,3,6);
plot(nz1(1,:));
title('nz1');

hold on

load fx2.mat; load  fy2.mat; load fz2.mat; 
load nx2.mat; load  ny2.mat; load nz2.mat; 

figure(6);
subplot(2,3,1);
plot(fx2(1,:));
title('fx2');

subplot(2,3,2);
plot(fy2(1,:));
title('fy2');

subplot(2,3,3);
plot(fz2(1,:));
title('fz2');

subplot(2,3,4);
plot(nx2(1,:));
title('nx2');

subplot(2,3,5);
plot(ny2(1,:));
title('ny2');

subplot(2,3,6);
plot(nz2(1,:));
title('nz2');

hold on

load fx3.mat; load  fy3.mat; load fz3.mat; 
load nx3.mat; load  ny3.mat; load nz3.mat; 

figure(7);
subplot(2,3,1);
plot(fx3(1,:));
title('fx3');

subplot(2,3,2);
plot(fy3(1,:));
title('fy3');

subplot(2,3,3);
plot(fz3(1,:));
title('fz3');

subplot(2,3,4);
plot(nx3(1,:));
title('nx3');

subplot(2,3,5);
plot(ny3(1,:));
title('ny3');

subplot(2,3,6);
plot(nz3(1,:));
title('nz3');

hold on

load Torque1.mat; load  Torque2.mat; load Torque3.mat; 

figure(8);
subplot(3,1,1);
plot(Torque1(1,:));
title('Torque1');

subplot(3,1,2);
plot(Torque2(1,:));
title('Torque2');

subplot(3,1,3);
plot(Torque3(1,:));
title('Torque3');
