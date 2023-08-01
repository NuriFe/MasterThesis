function plot_error_position(error,tstep,tend)
x = [];
y = [];
z = [];


x = error(:,1);
y = error(:,2);
z = error(:,3);
tout = 0:tstep:tend;

%figure();clf

 
x = x';
y = y';
z = z';
plot(tout,x);
hold on
plot(tout,y);
plot(tout,z);
legend('X_error', 'Y_error', 'Z_error');
ylabel('error (m)');
xlim([0,tend]);
ylim([-0.3 0.3]);


end