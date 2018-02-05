clear;
Data = csvread("../build/cte_data_021.csv");
figure;
plot(Data);
legend("cte", "p", "d", "i", "pid");
title("");