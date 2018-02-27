
data = read("build/Test.dat", -1, 2);
t = data(:, 2);
q = data(:, 1);

plot(t, q);
