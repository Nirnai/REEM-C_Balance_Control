zmp_filterd = csvread("ZMP_filtered.csv");
zmp_unfiltered = csvread("ZMP_unfiltered.csv");

zmp = zmp_filterd(:,1);
com = zmp_filterd(:,2);

hold on;
plot(zmp);
plot(com);


fs = 1000;
Y = fft(zmp);
Y = abs(Y/size(zmp,1));
Y = Y(1:size(zmp,1)/2+1,:);
Y(2:end-1,:) = 2*Y(2:end-1,:);
f = fs*(0:(size(zmp,1)/2))/size(zmp,1);


figure();
plot(f,Y) 

% Y1 = fft(y);
% Y1 = abs(Y1/size(y,1));
% Y1 = Y1(1:size(y,1)/2+1,:);
% Y1(2:end-1,:) = 2*Y1(2:end-1,:);
% f = fs*(0:(size(y,1)/2))/size(y,1);
% 
% 
% figure();
% plot(f,Y(:,1))
% hold on;
% plot(f,Y1) 