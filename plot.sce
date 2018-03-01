
data = read("/home/senserlex/RosWorkspaces/kuka_ws/test.dat", -1, 6);
q = data(:, 1:6);
//t = (1:size(data, 1))*0.004;
plot(q);
legend("$q_1$", "$q_2$", "$q_3$", "$q_4$", "$q_5$", "$q_6$")
