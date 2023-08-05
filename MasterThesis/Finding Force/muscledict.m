function muscles=muscledict(muscle)

switch muscle
    case 1 % triceps
        muscles=[129:133];%[104:108 129:133];
    case 2 % lat dorsi
        muscles=[90:95];
    case 3 % serratus anterior
        muscles=[25:36];
    case 4 % biceps/brachialis
        muscles=[83:85];
    case 5 % supra/infraspinatus
        muscles=[55:60 68:71];
    case 6 % rhomboids
        muscles=[20:24];
    case 7 % lower pectoralis
        muscles=[96:101];
    case 8 % upper pectoralis
        muscles=[102:103];
    case 9 % static
        muscles=[];
end

