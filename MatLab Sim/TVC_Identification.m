%%
%Loading the files:
in_data = load('simdata_tvc.txt');    % or readmatrix('input_data.txt');
out_data = load('simdata_realtvc.txt');

%Extract data:
u = in_data(:,2);
y = out_data(:,2);
Ts = 0.01; %Sampling rate

%Create iddata object:
data = iddata(y, u, Ts);
%%