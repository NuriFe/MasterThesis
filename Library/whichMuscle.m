function muscles=whichMuscles(muscle)

switch muscle
    case 1 % triceps
        muscles=[129:133];%[104:108 129:133];
    case 2 % deltoids
        muscles=[37:51];
    case 3 % biceps/brachialis
        muscles=[83:85];
    case 4 % static
        muscles=[];
end

