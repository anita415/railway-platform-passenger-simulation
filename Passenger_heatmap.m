clc;
clear;
close all;

% Folder containing the saved simulation states
folder = 'time_states_20260815-144104';
files = dir(fullfile(folder, 'agents_step_*.xml'));

% Sort the files into simulation step order
[~, idx] = sort({files.name});
files = files(idx);


all_x = [];
all_y = [];
wall_x = [];
wall_y = [];

for f = 1:length(files)
    xmlFile = fullfile(folder, files(f).name);
    doc = xmlread(xmlFile);
    agents = doc.getElementsByTagName('xagent');

    for k = 0:agents.getLength-1
        agent = agents.item(k);
        nameNode = agent.getElementsByTagName('name');

        if nameNode.getLength == 0
            continue;
        end

        agentName = strtrim(char(nameNode.item(0).getTextContent));

        xNode = agent.getElementsByTagName('x');
        yNode = agent.getElementsByTagName('y');

        if xNode.getLength == 0 || yNode.getLength == 0
            continue;
        end

        xVal = str2double(char(xNode.item(0).getTextContent));
        yVal = str2double(char(yNode.item(0).getTextContent));

        if strcmp(agentName, 'person')
            all_x(end+1) = xVal;
            all_y(end+1) = yVal;
        elseif strcmp(agentName, 'wall') && f == 1
            wall_x(end+1) = xVal;
            wall_y(end+1) = yVal;
        end
    end
end

binSize = 0.5;

xmin = floor(min(wall_x)) - 1;
xmax = ceil(max(wall_x)) + 1;
ymin = floor(min(wall_y)) - 1;
ymax = ceil(max(wall_y)) + 1;

xedges = xmin:binSize:xmax;
yedges = ymin:binSize:ymax;

% Exclude passenger spawning zones from hotspot analysis
enterance = ...
    all_x >= 18.5 & all_x <= 21.5 & ...
    all_y >= -1 & all_y <= 0.8;

train = ...
    all_x >= 2 & all_x <= 38 & ...
    all_y >= 7.4 & all_y <= 7.6;

% Keep all passenger positions except those inside spawning zones
keep = ~(enterance | train); 

all_x_plot = all_x(keep);
all_y_plot = all_y(keep);

[N, Xedges, Yedges] = histcounts2( ...
    all_x_plot, all_y_plot, xedges, yedges);

% Smooth the density map
%https://uk.mathworks.com/help/matlab/ref/smoothdata.html
N_smooth = smoothdata(N, 1, 'gaussian', 9);
N_smooth = smoothdata(N_smooth, 2, 'gaussian', 9);


figure;
imagesc(Xedges, Yedges, N_smooth');
set(gca, 'YDir', 'normal');
hold on;
set(gca, 'YDir', 'normal');
hold on;

plot(wall_x, wall_y, 'k.', 'MarkerSize', 5);

axis equal;
xlabel('x');
ylabel('y');
title('Passenger hotspot heatmap');
colorbar;
grid on;