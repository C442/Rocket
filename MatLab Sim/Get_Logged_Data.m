%%
% Extract both logged signals
signal1 = out.logsout{1}.Values;  % First signal
signal2 = out.logsout{2}.Values;  % Second signal

% Assume they share the same time vector:
time = signal1.Time;

% Extract signal data vectors
data1 = signal1.Data;
data2 = signal2.Data;

% Combine into one matrix: [time, data1, data2]
combined = [time, data1, data2];

% Write to CSV
csvwrite('sim_output_new2.csv', combined);  % or use writematrix (see below)

%%