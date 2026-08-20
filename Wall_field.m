clc;
clear;
close all;

folder = ['time_states_20260810-223301'];
xmlFile = fullfile(folder, 'agents_step_00000.xml');

doc = xmlread(xmlFile);
agents = doc.getElementsByTagName('xagent');

wall_x = [];
wall_y = [];
wall_m = [];
wall_n = [];
wall_p = [];

for k = 0:agents.getLength-1
    agent = agents.item(k);
    nameNode = agent.getElementsByTagName('name');

    if nameNode.getLength == 0
        continue;
    end

    agentName = strtrim(char(nameNode.item(0).getTextContent));

    if strcmp(agentName, 'wall')
        xNode = agent.getElementsByTagName('x');
        yNode = agent.getElementsByTagName('y');
        mNode = agent.getElementsByTagName('m');
        nNode = agent.getElementsByTagName('n');
        pNode = agent.getElementsByTagName('p');

        wall_x(end+1) = str2double(char(xNode.item(0).getTextContent));
        wall_y(end+1) = str2double(char(yNode.item(0).getTextContent));
        wall_m(end+1) = str2double(char(mNode.item(0).getTextContent));
        wall_n(end+1) = str2double(char(nNode.item(0).getTextContent));
        wall_p(end+1) = str2double(char(pNode.item(0).getTextContent));
    end
end

xmin = floor(min(wall_x)) - 2;
xmax = ceil(max(wall_x)) + 2;
ymin = floor(min(wall_y)) - 2;
ymax = ceil(max(wall_y)) + 2;

[xg, yg] = meshgrid(xmin:0.2:xmax, ymin:0.2:ymax);

field_total = zeros(size(xg));
cap = 1;

for a = 1:length(wall_x)
    dist = sqrt((xg - wall_x(a)).^2 + (yg - wall_y(a)).^2);
    field = min(wall_m(a) ./ (wall_n(a) + dist).^wall_p(a), cap);
    field_total = max(field_total, field);
end

figure;
surf(xg, yg, field_total, 'EdgeColor', 'none');
hold on;
plot3(wall_x, wall_y, max(field_total(:))*ones(size(wall_x)), ...
    'k.', 'MarkerSize', 8);

xlabel('x');
ylabel('y');
zlabel('Field strength');
title('Wall-induced social force field');
colorbar;
view(45,30);
grid on;