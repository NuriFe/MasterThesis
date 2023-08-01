function plot_neurexct(t,x,u)
muscles = containers.Map();

if nargin >2
    i=0;
    x=u;
else
    i=160;

end
figure;
xrange = [t(1) t(end)];
yrange = [0 1];

muscles('deltscap') = 37:1:47;
muscles('deltclav') = 48:1:51;

muscles('coracobr') = 53:1:54;
muscles('bic') = [83];
muscles('bicb') = 84:1:85;

muscles('triclong') = 86:1:8;
muscles('tricmed') = 104:1:108;
muscles('triclat') = 129:1:133;

muscles('pectmin') = 16:1:19;
muscles('pectmaj_thor') = 96:1:101;
muscles('pectmaj_clav') = 103:1:103;

muscles('brachialis') = 109:1:115;
muscles('brachiorad') = 116:1:118;

deltscap = mean(x(:,muscles('deltscap')+i),2);
deltclav = mean(x(:,muscles('deltclav')+i),2);

subplot(2,3,1)
plot(t,[deltscap,deltclav]);
legend('Deltoid Scapular','Deltoid Clavicular');
xlabel('time (s)');
ylabel('neural excitation');
axis([ xrange yrange]);

coracobr = mean(x(:, muscles('coracobr')+i), 2);
brachialis = mean(x(:, muscles('brachialis')+i), 2);
brachiorad = mean(x(:, muscles('brachiorad')+i), 2);

subplot(2,3,2)
plot(t,[coracobr, brachialis, brachiorad]);
legend('Coracobrachialis','Brachialis','Brachioradialis');
xlabel('time (s)');
ylabel('neural excitation');
axis([ xrange yrange]);

bic = mean(x(:, muscles('bic')+i), 2);
bicb = mean(x(:, muscles('bicb')+i), 2);

subplot(2,3,3)
plot(t,[bic, bicb]);
legend('Biceps, long head','Biceps, short head');
xlabel('time (s)');
ylabel('neural excitation');
axis([ xrange yrange]);

triclong = mean(x(:, muscles('triclong')+i), 2);
tricmed = mean(x(:, muscles('tricmed')+i), 2);
triclat = mean(x(:, muscles('triclat')+i), 2);


subplot(2,3,4)
plot(t,[triclong, tricmed, triclat]);
legend('Triceps, long head','Triceps, medial head','Triceps, lateral head');
xlabel('time (s)');
ylabel('neural excitation');
axis([ xrange yrange]);

pectmin = mean(x(:, muscles('pectmin')+i), 2);
pectmaj_thor = mean(x(:, muscles('pectmaj_thor')+i), 2);
pectmaj_clav = mean(x(:, muscles('pectmaj_clav')+i), 2);

subplot(2,3,5)
plot(t,[pectmin, pectmaj_thor, pectmaj_clav]);
legend('Pectoralis minor','Pectoralis major thoracic','Pectoralis major clavicular');
xlabel('time (s)');
ylabel('neural excitation');
axis([ xrange yrange]);

end