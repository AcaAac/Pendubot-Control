data = out.lyapunov.signals.values;
writematrix(data, 'lyapunov_data.csv');
data2 = out.time.signals.values;
writematrix(data2, 'time_data.csv');
data3 = out.dV.signals.values;
writematrix(data3, 'dv_data.csv');
