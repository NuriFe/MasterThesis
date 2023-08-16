a = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\ToCheck\960_to_check.mat');
b = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\ToCheck\101_200_to_check.mat');
c = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\ToCheck\201_250_to_check.mat');
d = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\ToCheck\251_260_to_check.mat');
e = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\ToCheck\261_300_to_check.mat');
f = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\ToCheck\301_960_to_check.mat');

tocheck = [a.tocheck b.tocheck c.tocheck d.tocheck e.tocheck f.tocheck];

w_refs = create_grid();
[model, nstates, ndof, nmus, iLce] = initialize_model();
plot_wrist_references(w_refs,model,tocheck');