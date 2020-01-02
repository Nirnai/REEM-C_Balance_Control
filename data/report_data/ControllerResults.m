%% individual plot
file = "LQRControl.csv";
data = csvread(file);

p = data(:,1);
x = data(:,2);
% p_ref = data(:,3);
t = 1:size(x);
dt = 0.001;
t = t*dt;

[maxP, idxP] = max(p(500:end));
[maxX, idxX] = max(x);
minX = 0.1 * maxX;
k = find(x(idxX:end) < minX, 1);
k = k + idxX;



startTime = 10000 * dt;
riseTime = (idxX - 10000) * dt;
fallTime = (k -idxX) * dt;

maxtxt = ['max = ',num2str(maxP,3),'\rightarrow'];
mintxt = ['settled = ',num2str(minX,3),'\rightarrow'];

pValTxt = ['max p = ', num2str(maxP,2),'m'];
xValTxt = ['max x = ', num2str(maxX,2),'m'];
rTimeTxt = ['rise time = ',num2str(riseTime,3),'s'];
fTimeTxt = ['settle time = ',num2str(fallTime,3),'s'];

figure('units', 'centimeters', 'pos', [0 0 15 10])
hold on;
    
text(0.3,0.2,pValTxt,'FontSize',8)
text(0.3,0.2-0.02,xValTxt,'FontSize',8)
text(0.3,0.2-0.04,rTimeTxt,'FontSize',8)
text(0.3,0.2-0.06,fTimeTxt,'FontSize',8)

plot(t,p)
text(startTime+riseTime,maxP,maxtxt, 'FontSize',8, 'HorizontalAlignment', 'right');
text(startTime+riseTime+fallTime,minX,mintxt, 'FontSize',8, 'HorizontalAlignment', 'right');
plot(t,x)
% plot(t,p_ref)

h1 = line([0 20],[0.105 0.105],'Color','green','LineStyle','--');
h2 = line([0 20],[-0.105 -0.105],'Color','green','LineStyle','--');
legend('p','x','SB boundry')
ylim([-0.21, 0.21]);
xlabel('t in [s]')
hold off;

%% Load Data

% Controller1 == ZMP simple Feedback
% Controller2 == ZMP CoM acceleration
% Controller3 == Ankle stratagy com


files = [
         "NoControl.csv"
         "PControl.csv"
         "LQRControl.csv"
         "CPControl.csv"
         ];
     
data = cell(size(files));
p = cell(size(files));
x = cell(size(files));
     
for i = 1:size(files)
    data{i} = csvread(files(i));
    p{i} = data{i}(:,1);
    x{i} = data{i}(:,2);
    if(size(data{i},2) > 2)
        p_ref = data{i}(:,3);
    end
end

t = 1:size(p{1});
dt = 0.001;
t = t*dt;

cols = 2;
rows = size(files,1)/cols;
figure


for i=1:rows*cols
    subplot(rows,cols,i);
    name = replace(files(i), ["_", ".csv", "state"], " ");
    title(name)
    hold on; 
    plot(t,p{i})
    plot(t,x{i})
    if(size(data{i},2) > 2)
        plot(t,p_ref);
    end
    h1 = line([0 20],[0.105 0.105],'Color','green','LineStyle','--');
    h2 = line([0 20],[-0.105 -0.105],'Color','green','LineStyle','--');
%     if(size(data{i},2) > 2)
%         legend('p','x', 'p_{ref}','SB boundry')
%     else
%         legend('p','x','SB boundry')
%     end
    ylim([-0.21, 0.21]);
    xlabel('t in [s]')
    hold off;
end


