function plot_error(t,error)
x = [];
y = [];
z = [];


x = error(:,1);
y = error(:,2);
z = error(:,3);

figure(1);clf

x = x';
y = y';
z = z';
subplot(3,1,1)
plot(t,x);
legend('X_error');
xlabel('time (s)');
ylabel('error (m)');


subplot(3,1,2)
plot(t,y);
legend('Y_error');
xlabel('time (s)');
ylabel('error (m)');

subplot(3,1,3)
plot(t,z);
legend('Z_error');
xlabel('time (s)');
ylabel('error (m)');
end