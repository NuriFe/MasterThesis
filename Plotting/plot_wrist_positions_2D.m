function plot_wrist_positions_2D(t,xout)
x = [];
y = [];
z = [];

xrange = [-0.3 0.6];
yrange = [-1 0.5];
zrange = [-0.35 0.35];


for i = 1:size(xout,1)
        state = xout(i,:)';
        pos = wrist_position(state);
        y = [y pos(3)];
        z = [z pos(1)];
        x = [x pos(2)];
end
figure(1);clf

t = t(1:end);
x = x';
y = y';
z = z';
subplot(3,1,1)
plot(t,x);
legend('X');
xlabel('time (s)');
ylabel('position (m)');


subplot(3,1,2)
plot(t,y);
legend('Y');
xlabel('time (s)');
ylabel('position (m)');

subplot(3,1,3)
plot(t,z);
legend('Z');
xlabel('time (s)');
ylabel('position (m)');



end