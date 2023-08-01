function plot_hand_forces(forces)
x = [];
y = [];
z = [];


x = forces(1,:);
y = forces(2,:);
z = forces(3,:);

%figure();clf

 
x = x';
y = y';
z = z';
plot(x);
hold on
plot(y);
plot(z);
legend('Fx', 'Fy', 'Fz');
ylabel('forces(N)');
xlim([1, length(x)]);




end