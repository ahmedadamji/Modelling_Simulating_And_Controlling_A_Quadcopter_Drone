%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%To clear the workspace and old variables upon each test run.
clc
clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 15;
spaceLimits = [-spaceDim/10 spaceDim -spaceDim/10 spaceDim/2 0 spaceDim/1.5]*1.5;

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

%figure to display position
f2 = figure;
ax2 = gca;
view(ax2,2);
grid ON
grid MINOR
%caxis(ax2, [0 spaceDim]);
hold(ax2,'on')
axis auto

%figure to display orientation
f3 = figure;
ax3 = gca;
view(ax3,2);
grid ON
grid MINOR
%caxis(ax3, [0 spaceDim]);
hold(ax3,'on')
axis auto

% %figure to display linear speed
% f4 = figure;
% ax4 = gca;
% view(ax4,2);
% grid ON
% grid MINOR
% %caxis(ax4, [0 spaceDim]);
% hold(ax4,'on')
% axis auto
% %ax4=0;

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    % Please change the name of the class here to reflect the appropriate
    % version of quadcopter simulation to run according to question.
    drones = [drones Drone_q1(ax1, ax2, ax3, spaceDim, num_drones)];
end

while(drones(1).time < 1000.0)
    clear axis
    cla(ax1);
    % Do not want graph to go away
    %cla(ax2);
    %cla(ax3);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow

    pause(0.1)
end
