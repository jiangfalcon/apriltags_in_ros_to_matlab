%% open hardware
clear all ; close all ; clc;

fprintf('Initializing hardware...\n');

% Connect to ROS master
setenv('ROS_MASTER_URI','http://192.168.1.94:11311')
setenv('ROS_IP','192.168.1.80')
rosinit; % for testing

tag_sub = rossubscriber('/camera/tag_detections');
camera_info_sub = rossubscriber('/left_cam/camera_info');
color_point_sub = rossubscriber('/color_point');
% tag_msg = rosmessage(tag_sub);
fprintf('Done.\n');  
%%
% struct declare
num = struct('id',[],'tag_dection',[]);
color = struct('id','tag_dection');

%%
camera_info_data = receive(camera_info_sub, 10);
fx = camera_info_data.K(1);
fy = camera_info_data.K(5);    
color = ['ro'; 'yo'; 'ko'; 'bo'];
color_arrow = ['r'; 'y'; 'k'; 'b'];
teg0_check = 0;

while 1;    
    fprintf('----------------------------------------\n'); 
    % receive topics
    tag_data = receive(tag_sub, 10);
    color_point_data = receive(color_point_sub, 10);
    
    dections = tag_data.Detections;
    num.tag_dection = size(dections,1)
    
    % avoid the interruption caused by zero tag dection
    if num.tag_dection == 0 
        continue;
    end
    
    % extract massages
    for i = 1:num.tag_dection
    id = dections(i).Id;
    num.id = id + 1;
    
    % extract pixel coordinate of apriltags
    pixcoord_tag{num.id}(1) = round(dections(i).Pixcoord.X);
    pixcoord_tag{num.id}(2) = camera_info_data.Height - round(dections(i).Pixcoord.Y); % set the pixel coordinate of tag0 and make the lower-left corner to be (0,0)
    
    % extract pixel coordinate of apriltags
    pos{num.id}(1) = dections(i).Pose.Pose.Position.X;
    pos{num.id}(2) = -dections(i).Pose.Pose.Position.Y;
    pos{num.id}(3) = dections(i).Pose.Pose.Position.Z;
    
    quat{num.id}(1) = dections(i).Pose.Pose.Orientation.X;
    quat{num.id}(2) = dections(i).Pose.Pose.Orientation.Y;
    quat{num.id}(3) = dections(i).Pose.Pose.Orientation.Z;
    quat{num.id}(4) = dections(i).Pose.Pose.Orientation.W;
    
    % calculate Euler angles
    eulZYX{num.id} = -quat2eul(quat{num.id});
    
    
    figure(1)
    % plot tags
    %plot3(pos{num.id}(1),pos{num.id}(2),pos{num.id}(3),color(num.id,:)); % for 3-d
    plot(pos{num.id}(1),pos{num.id}(2),color(num.id,:)); % for 2-d
    hold on 
    
    % plot orientation of tags
    u = 0.05*cos(eulZYX{num.id}(3));
    v = 0.05*sin(eulZYX{num.id}(3));  
    h = quiver(pos{num.id}(1),pos{num.id}(2),u,v,color_arrow(num.id,:),'Linewidth',3);
    set(h,'maxheadsize',1);
    hold on
    
    % plot color point
    if color_point_data.Point.Z == 1
        px = color_point_data.Point.X;
        py = double(camera_info_data.Height) - color_point_data.Point.Y;
    else
        px = [];
        py = [];
    end
    
    
    if i == num.tag_dection 
        for j = 1:num.tag_dection
            teg0_check = teg0_check || dections(j).Id == 0;
        end
        if teg0_check == 1
            Pz_temp = 0;
            for k = 1:num.tag_dection
                id = dections(k).Id;
                num.id = id + 1;
                Pz_temp = Pz_temp + pos{num.id}(3);
            end
            Pz = Pz_temp/num.tag_dection;
            Px = Pz*(px - pixcoord_tag{1}(1))/fx + pos{1}(1);
            Py = Pz*(py - pixcoord_tag{1}(2))/fy + pos{1}(2);
            plot(Px,Py,'g.','MarkerSize',20);
            teg0_check = 0; % reset teg0_check
        end
    end
    
    % plot camera
    %plot3(0,0,0,'k.','MarkerSize',40);
    grid on
    axis([-0.3 0.3 -0.3 0.3]); % for 2-d
    %axis([-0.3 0.3 -0.3 0.3 0 1]); % for 3-d
    axis square
    %view([0,0,1]); % for 3-d
    %view([1,-0.5,0.5]); % for 3-d
    xlabel('x');ylabel('y');zlabel('z');
    hold on
    end
    hold off
    %getframe;
    %pause(0.01)
end


%% close hardware
% Disconnect ROS master
rosshutdown;
clear all ; close all ; clc;

