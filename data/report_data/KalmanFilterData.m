file = "stateEstimation.csv";
data = csvread(file);

dt = 0.001;

x = data(:,1);
v = data(:,2);
a = data(:,3);
x_hat = data(:,4);
v_hat = data(:,5);
a_hat = data(:,6);
x_model = data(:,7);
a_model = data(:,8);


%%

figure;
hold on;
plot(x)
plot(x_hat)
% plot(x_model)
legend('x','x_{kalman}')
ylim([-0.1, 0.1])


figure;
hold on;
plot(a)
plot(a_hat)
% plot(a_model)
leg = legend('$\ddot{x}$','$\ddot{x}_{kalman}$');
set(leg,'Interpreter','latex');
ylim([-1, 1])

%%


file = "stateEstimation2.csv";
data = csvread(file);

dt = 0.001;

x = data(:,1);
v = data(:,2);
a = data(:,3);
x_hat1 = data(:,4);
v_hat1 = data(:,5);
a_hat1 = data(:,6);
x_hat2 = data(:,7);
v_hat2 = data(:,8);
a_hat2 = data(:,9);

test1 = x-mean(x);
test2 = x_hat1-mean(x_hat1);

figure
hold on
plot(x)
plot(x_hat1)
% plot(x_hat2)

figure
hold on
plot(v)
plot(v_hat1)
% plot(v_hat2)

figure
hold on
plot(a)
plot(a_hat1)
% plot(a_hat2)







%%




file = "JerkTest.csv";
data = csvread(file);


p = data(:,1);
x = data(:,2);
x_hat = data(:,3);
v_hat = data(:,4);
a_hat = data(:,5);
x_com = data(:,6);
v_com = data(:,7);
a_com = data(:,8);
j_com = data(:,9);

hold on;
plot(j_com)
plot(x_hat)
plot(v_hat)
plot(a_hat)





















%%
fc = 10;
fs = 1000;

[b,a] = butter(6,fc/(fs/2));
freqz(b,a)

a_test = filter(b,a,xdd);

%% 


Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 10000;             % Length of signal
Y = fft(a);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
