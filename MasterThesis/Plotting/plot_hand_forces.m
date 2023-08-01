function plot_hand_forces(forces,tstep, tend)
x = [];
y = [];
z = [];


x = forces(1,:);
y = forces(2,:);
z = forces(3,:);

tout = 0:tstep:tend;
%figure();clf


x = x';
y = y';
z = z';
tout = tout';
plot(tout,x);
hold on
plot(tout,y);
plot(tout,z);
legend('Fx', 'Fy', 'Fz');
ylabel('forces(N)');
xlim([0, tend]);
ylim([-100 100]);



end