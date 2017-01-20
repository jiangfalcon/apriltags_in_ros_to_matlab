%% open hardware
clear all ; close all ; clc;

fprintf('Initializing hardware...\n');

% Connect to ROS master
setenv('ROS_MASTER_URI','http://192.168.1.94:11311')
setenv('ROS_IP','192.168.1.80')
rosinit; % for testing

tag_sub = rossubscriber('/camera/tag_detections');
tag_msg = rosmessage(tag_sub);
fprintf('Done.\n');  
%%
% struct declare
num = struct('id',[],'tag_dection',[]);
color = struct('id','tag_dection')

%%
color = ['ro'; 'go'; 'ko'; 'bo'];

posx = zeros(1,3);
posy = zeros(1,3);
posz = zeros(1,3);
while 1;    
    fprintf('-------------------------------------------\n'); 
    tag_data = receive(tag_sub, 10);
    dections = tag_data.Detections;
    num.tag_dection = size(dections,1)
    
    if num.tag_dection == 0 
        continue;
    end
    
    for i = 1:num.tag_dection
    id = dections(i).Id;
    num.id = id + 1;
    posx(num.id) = dections(i).Pose.Pose.Position.X;
    posy(num.id) = dections(i).Pose.Pose.Position.Y;
    posz(num.id) = dections(i).Pose.Pose.Position.Z;
    
    figure(1)
    %plot tags
    plot3(posx(num.id),posy(num.id),posz(num.id),color(num.id,:));
    hold on 
    
    %plot camera
    plot3(0,0,0,'k.','MarkerSize',40);
    grid on
    axis([-1 1 -1 1 0 4]);
    view([1,-0.5,0.5]);
    xlabel('x');ylabel('y');zlabel('z');
    hold on
    end
    hold off
   % getframe;
    %pause(0.01)
end


%% close hardware
% Disconnect ROS master
rosshutdown;
clear all ; close all ; clc;

%% stop the car
cmd_msg.Linear.X = 0;
cmd_msg.Angular.Z = 0;
send(cmd_pub, cmd_msg);
