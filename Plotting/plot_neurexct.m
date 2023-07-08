function plot_neurexct(t,x)
muscles = containers.Map();
xrange = [0 1];
yrange = [0 1];
zrange = [0 1];

figure;
t = t(3:end);

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

deltscap = mean(x(3:end,muscles('deltscap')+160),2);
deltclav = mean(x(3:end,muscles('deltclav')+160),2);

subplot(2,3,1)
plot(t,[deltscap,deltclav]);
legend('Deltoid Scapular','Deltoid Clavicular');
xlabel('time (s)');
ylabel('neural excitation');
axis([zrange xrange yrange]);

coracobr = mean(x(3:end, muscles('coracobr')+160), 2);
brachialis = mean(x(3:end, muscles('brachialis')+160), 2);
brachiorad = mean(x(3:end, muscles('brachiorad')+160), 2);

subplot(2,3,2)
plot(t,[coracobr, brachialis, brachiorad]);
legend('Coracobrachialis','Brachialis','Brachioradialis');
xlabel('time (s)');
ylabel('neural excitation');
axis([zrange xrange yrange]);

bic = mean(x(3:end, muscles('bic')+160), 2);
bicb = mean(x(3:end, muscles('bicb')+160), 2);

subplot(2,3,3)
plot(t,[bic, bicb]);
legend('Biceps, long head','Biceps, short head');
xlabel('time (s)');
ylabel('neural excitation');
axis([zrange xrange yrange]);

triclong = mean(x(3:end, muscles('triclong')+160), 2);
tricmed = mean(x(3:end, muscles('tricmed')+160), 2);
triclat = mean(x(3:end, muscles('triclat')+160), 2);


subplot(2,3,4)
plot(t,[triclong, tricmed, triclat]);
legend('Triceps, long head','Triceps, medial head','Triceps, lateral head');
xlabel('time (s)');
ylabel('neural excitation');
axis([zrange xrange yrange]);

pectmin = mean(x(3:end, muscles('pectmin')+160), 2);
pectmaj_thor = mean(x(3:end, muscles('pectmaj_thor')+160), 2);
pectmaj_clav = mean(x(3:end, muscles('pectmaj_clav')+160), 2);

subplot(2,3,5)
plot(t,[pectmin, pectmaj_thor, pectmaj_clav]);
legend('Pectoralis minor','Pectoralis major thoracic','Pectoralis major clavicular');
xlabel('time (s)');
ylabel('neural excitation');
axis([zrange xrange yrange]);

end