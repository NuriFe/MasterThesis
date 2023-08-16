function plot_wrist_positions_2D(t,xout,goal)
x = [];
y = [];
z = [];

for i = 1:size(xout,1)
        state = xout(i,:)';
        pos = wrist_position(state);
        y = [y pos(3)];
        z = [z pos(1)];
        x = [x pos(2)];
end

t = t(1:end);
x = x';
y = y';
z = z';
subplot(3,1,1)
plot(t,x);
hold on
hline = refline([0 goal(2)]);
hline.Color = 'r';
% if goal(2)< 0
%     ylim([1.5*goal(2), 0.5*goal(2)]);
% else
%     ylim([0.5*goal(2), 1.5*goal(2)]);
% 
% end
% if abs(goal(2))<0.05
%     ylim([-0.05,0.05]);
% end
legend('X');
xlabel('time (s)');
ylabel('position (m)');
hold off

subplot(3,1,2)
plot(t,y);
hold on
hline = refline([0 goal(3)]);
hline.Color = 'r';
% if goal(3)< 0
%     ylim([1.5*goal(3), 0.5*goal(3)]);
% else
%     ylim([0.5*goal(3), 1.5*goal(3)]);
% end
% if abs(goal(3))<0.05
%     ylim([-0.05,0.05]);
% end
legend('Y');
xlabel('time (s)');
ylabel('position (m)');
hold off

subplot(3,1,3)
plot(t,z);
hold on
hline = refline([0 goal(1)]);
hline.Color = 'r';
% if goal(1)< 0
%     ylim([1.5*goal(1), 0.5*goal(1)]);
% else
%     ylim([0.5*goal(1), 1.5*goal(1)]);
% 
% end
% if abs(goal(1)) <0.05
%     ylim([-0.05,0.05]);
% end
legend('Z');
xlabel('time (s)');
ylabel('position (m)');
hold off



end