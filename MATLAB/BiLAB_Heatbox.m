clc; clear all; close all;

%% PID test analysis

PATH = "/Users/MarcusTS/Library/Application Support/JetBrains/PyCharmEdu2022.2/scratches/Lab Meeting Tests/";
%PATH = "/Users/MarcusTS/Library/Application Support/JetBrains/PyCharmEdu2022.2/scratches/";
excel_folder = PATH;
cd(PATH);
filetype='Heat_Test*.xlsx';
f = fullfile(excel_folder, filetype);
file_names = dir(f);

cohenC = true;

% Gather all data from test files in cell format
readData = cell(length(file_names), 1);
for ii = 1:length(file_names)
    tempMatrix = readmatrix(file_names(ii).name);
    tempCell =readcell(file_names(ii).name);
    readData{ii,1} = tempMatrix(:,1); % Time data
    readData{ii,2} = tempMatrix(:,2); % Temp 1 data
    readData{ii,3} = tempMatrix(:,3); % Temp 2 data
    readData{ii,4} = tempMatrix(:,4); % Power data
    readData{ii,5} = tempMatrix(1,5); % Kp value
    readData{ii,6} = tempMatrix(1,6); % Ki value
    readData{ii,7} = tempMatrix(1,7); % Kd value
    if(size(tempCell,2) > 7)
        readData{ii,8} = tempCell(1:5,8:end); % Remaining values/notes 
    end
    readData{ii,9} = file_names(ii).date; % Last update-of-file-date, not actual date of test, NOT ACTUAL DATE
    if cohenC && (size(tempMatrix,2) >= 14)
        readData{ii,10} = tempMatrix(1,14);
        readData{ii,11} = tempMatrix(1,16);
    end
end

%% Sorting of tests based on: PID values

% PID values
kp = 55;
ki = 0.2;
kd = 29.1;

PIDsortedTestNBR = [];
PIDsorted = cell(size(readData,1),size(readData,2));

cnt = 1;
for i = 1:length(readData)
    if (readData{i,5} == kp && readData{i,6} == ki && readData{i,7} == kd)
        PIDsorted(cnt,:) = readData(i,:);
        PIDsortedTestNBR(cnt,1) = i;
        cnt = cnt + 1;
    end
end

pidNonEmpty = ~cellfun('isempty',PIDsorted);
filledRows = sum(pidNonEmpty);
PIDsortedClean = cell(filledRows(1,1),size(PIDsorted,2));
PIDsortedClean = PIDsorted(1:filledRows(1,1),:);
PIDsorted = PIDsortedClean;



%% Set Point-organized

tempSorted = struct();
sp = nan(length(readData),1);

for i = 1:length(readData)
    if isempty(readData{i,8})
        continue;
    end
    if (readData{i,8}{1,1} == "Set Point")
        sp(i,1) = readData{i,8}{2,1};
    end
end
sps = unique(sp);
sps = sps(~isnan(sps));

for i = 1:length(sps)
    tempconst = sps(i);
    tempstring = sprintf('%.0f',tempconst);
    structText = append('T',tempstring);

    ite = find(sp==tempconst);
    for ii = 1:length(ite)
        for j=1:size(readData,2)
            tempSorted.(structText){ii,j} = readData{ite(ii),j};
        end
    end
end


%% Evaluate 

% SAKNAS SET POINT TEMP: Feb 07-Feb14 


specTests = [2, 4, 7, 9, 11];

CohenCvalues = zeros(length(specTests), 8);

%for i=1:length(specTests)
 %   TEST_NBR = specTests(i);
for i=1:length(readData)
    TEST_NBR = i;



    % cohen coon tests: 2,4,7,9,11
    %TEST_NBR = 11;

    xTime = readData{TEST_NBR,1}(:,1);
    yTemp1 = readData{TEST_NBR,2}(:,1);
    yTemp2 = readData{TEST_NBR,3}(:,1);
    yPower = readData{TEST_NBR,4}(:,1);
    pidVals = readData(TEST_NBR,5:7);
    setPoint = readData{TEST_NBR,8}{2,1};
    comments = readData{TEST_NBR,8}(1:2,3:end);
    date = readData{TEST_NBR,9};
    
    figure;
    subplot(3,1,[1 2])
    hold on;
    title(date,'FontSize',14)
    plot(xTime, yTemp1, 'b', 'LineWidth', 2);
    %plot(xTime, yTemp2, 'r', 'LineWidth', 2);
    yline(setPoint, 'k', 'LineWidth', 2);
    %legend('Temp 1', 'Temp 2', 'Set Point');
    legend('Temp 1', 'Set Point');

    subplot(3,1,3)
    plot(xTime, yPower, 'k', 'LineWidth', 2);
    legend('Dim Power');
    hold off;
end



%% Find max slope 

tempCCvalues = zeros(7,8);

for j = 1:size(tempCCvalues,1)

P_INTERVAL = 50+20*j;
T_START = 0;
T_END = xTime(length(xTime))-10; % varför begränsa punkter?          

ind = find(T_START <= xTime & xTime <= T_END);
time_slope = xTime(ind);
temp1_slope = yTemp1(ind);

temp1_min = min(temp1_slope);
temp1_max = max(temp1_slope);
pv63 = temp1_min + 0.63*(temp1_max - temp1_min);

figure('Name', 'Cohen Coon')
hold on
plot(time_slope,temp1_slope)
yline(temp1_slope(1), 'k', 'LineWidth', 1);
yline(temp1_slope(length(temp1_slope)-1), 'k', 'LineWidth', 1);
yline(pv63, 'k', 'LineWidth', 1);


slope_MAX = 0; % Set to 0 as base minimum
time_stamp = 0;
dy_stamp = 0;
dx_stamp = 0;

for ii = 1:(length(temp1_slope)-P_INTERVAL)
    dy = temp1_slope(ii+P_INTERVAL) - temp1_slope(ii);
    dx = time_slope(ii+P_INTERVAL) - time_slope(ii);
    k = dy/dx;
    if k > slope_MAX
        slope_MAX = k;
        time_stamp = time_slope(ii + (P_INTERVAL/2));
        slope_m = temp1_slope(ii + (P_INTERVAL/2)) - k*time_stamp;
        dy_stamp = dy;
        dx_stamp = dx;
        
    end
end

slope_ys_all = slope_MAX.*time_slope + slope_m;
slope_63ind = find(((temp1_min-0.005*temp1_min) <= slope_ys_all & (pv63 + 0.005*pv63) >= slope_ys_all));
slope_y = slope_ys_all(slope_63ind);
slope_x = time_slope(slope_63ind);

[~,closest63ind] = min(abs(slope_y-pv63));
[~,closestMINind] = min(abs(slope_y-temp1_min));

plot(slope_x,slope_y)
scatter(slope_x(closest63ind), slope_y(closest63ind),'filled');
scatter(slope_x(closestMINind), slope_y(closestMINind),'filled');
hold off;   


startP = readData{TEST_NBR,10};
stepP = readData{TEST_NBR,11}; 

Td = slope_x(closestMINind) - time_slope(1) + 15; % +15 to compensate for starting time of program when changing power step variable
Tau = slope_x(closest63ind) - slope_x(closestMINind);
PVchange = (temp1_max - temp1_min)/temp1_min;
COchange = (stepP-startP)/startP; 
gp = PVchange / COchange;


% P Controller Values:
Kp_p = (1.03/gp) * ((Tau/Td) + 0.34);

% PI Controller Values:
Kp_pi = (0.9/gp) * ((Tau/Td) + 0.092);
Ki_pi = 3.33 * Td * ((Tau + (0.0092 * Td)) / (Tau + (2.22 * Td)));

% PD Controller Values:
Kp_pd = (1.24/gp) * ((Tau/Td) + 0.129);
Kd_pd = 0.27 * Td * ((Tau - (0.324 * Td)) / (Tau + (0.129 * Td)));

% PID Controller Values:
Kp_pid = (1.35/gp) * ((Tau/Td) + 0.185);
Ki_pid = 2.5 * Td * ((Tau + (0.185 * Td)) / (Tau + (0.611 * Td)));
Kd_pid = 0.37 * Td * (Tau / (Tau + (0.129 * Td)));

tempCCvalues(j,1) = Kp_p;
tempCCvalues(j,2) = Kp_pi;
tempCCvalues(j,3) = Ki_pi;
tempCCvalues(j,4) = Kp_pd;
tempCCvalues(j,5) = Kd_pd;
tempCCvalues(j,6) = Kp_pid;
tempCCvalues(j,7) = Ki_pid;
tempCCvalues(j,8) = Kd_pid;

end

avgCCvals = mean(tempCCvalues);
CohenCvalues(i,:) = avgCCvals(1,:)

% disp('P Controller Values: ')
% disp(['Kp = ',num2str(Kp_p),''])
% disp('PI Controller Values: ')
% disp(['Kp = ',num2str(Kp_pi),''])
% disp(['Ki = ',num2str(Ki_pi),''])
% disp('PD Controller Values: ')
% disp(['Kp = ',num2str(Kp_pd),''])
% disp(['Kd = ',num2str(Kd_pd),''])
% disp('PID Controller Values: ')
% disp(['Kp = ',num2str(Kp_pid),''])
% disp(['Ki = ',num2str(Ki_pid),''])
% disp(['Kd = ',num2str(Kd_pid),''])
% disp(" ")


%end

meanCCvals_final = mean(CohenCvalues);
Kp_p = meanCCvals_final(1,1);
Kp_pi = meanCCvals_final(1,2);
Ki_pi = meanCCvals_final(1,3);
Kp_pd = meanCCvals_final(1,4);
Kd_pd = meanCCvals_final(1,5);
Kp_pid = meanCCvals_final(1,6);
Ki_pid = meanCCvals_final(1,7);
Kd_pid = meanCCvals_final(1,8);
table(Kp_p,Kp_pi,Ki_pi,Kp_pd,Kd_pd,Kp_pid,Ki_pid,Kd_pid)

