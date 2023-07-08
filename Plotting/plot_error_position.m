function plot_error_position(error)
x = [];
y = [];
z = [];


x = error(:,1);
y = error(:,2);
z = error(:,3);

%figure();clf

 
x = x';
y = y';
z = z';
plot(x);
hold on
plot(y);
plot(z);
legend('X_error', 'Y_error', 'Z_error');
ylabel('error (m)');
xlim([1, length(x)]);



end