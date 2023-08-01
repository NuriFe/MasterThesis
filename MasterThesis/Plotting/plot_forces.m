function plot_forces(t,mf)
muscles = containers.Map();

figure;
t = t(2:end);

muscles('deltscap') = 37:1:47;
muscles('deltclav') = 48:1:51;

muscles('coracobr') = 52:1:54;
muscles('bic') = [83];
muscles('bicb') = 84:1:85;

muscles('triclong') = 86:1:8
muscles('tricmed') = 104:1:108;
muscles('triclat') = 129:1:133;

muscles('pectmin') = 16:1:19;
muscles('pectmaj_thor') = 96:1:101;
muscles('pectmaj_clav') = 102:1:103;

muscles('brachialis') = 109:1:115;
muscles('brachiorad') = 116:1:118;

deltscap = mean(mf(2:end,muscles('deltscap')),2);
deltclav = mean(mf(2:end,muscles('deltclav')),2);

subplot(2,3,1)
plot(t,[deltscap,deltclav]);
legend('Deltoid Scapular','Deltoid Clavicular');
xlabel('time (s)');
ylabel('forces (N)');

coracobr = mean(mf(2:end, muscles('coracobr')), 2);
brachialis = mean(mf(2:end, muscles('brachialis')), 2);
brachiorad = mean(mf(2:end, muscles('brachiorad')), 2);

subplot(2,3,2)
plot(t,[coracobr, brachialis, brachiorad]);
legend('Coracobrachialis','Brachialis','Brachioradialis');
xlabel('time (s)');
ylabel('forces (N)');

bic = mean(mf(2:end, muscles('bic')), 2);
bicb = mean(mf(2:end, muscles('bicb')), 2);

subplot(2,3,3)
plot(t,[bic, bicb]);
legend('Biceps, long head','Biceps, short head');
xlabel('time (s)');
ylabel('forces (N)');

triclong = mean(mf(2:end, muscles('triclong')), 2);
tricmed = mean(mf(2:end, muscles('tricmed')), 2);
triclat = mean(mf(2:end, muscles('triclat')), 2);


subplot(2,3,4)
plot(t,[triclong, tricmed, triclat]);
legend('Triceps, long head','Triceps, medial head','Triceps, lateral head');
xlabel('time (s)');
ylabel('forces (N)');

pectmin = mean(mf(2:end, muscles('pectmin')), 2);
pectmaj_thor = mean(mf(2:end, muscles('pectmaj_thor')), 2);
pectmaj_clav = mean(mf(2:end, muscles('pectmaj_clav')), 2);

subplot(2,3,5)
plot(t,[pectmin, pectmaj_thor, pectmaj_clav]);
legend('Pectoralis minor','Pectoralis major thoracic','Pectoralis major clavicular');
xlabel('time (s)');
ylabel('forces (N)');


end