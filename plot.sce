
data = read("build/Test.dat", -1, 7);
q = data(:, 1:6);
t = data(:, 7);

plot(t, q);
legend("$q_1$", "$q_2$", "$q_3$", "$q_4$", "$q_5$", "$q_6$")
