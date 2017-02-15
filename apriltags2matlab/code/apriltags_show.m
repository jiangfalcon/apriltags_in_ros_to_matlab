%% open hardware
clear all ; close all ; clc;

fprintf('Initializing hardware...\n');

% Connect to ROS master
setenv('ROS_MASTER_URI','http://192.168.1.94:11311')
setenv('ROS_IP','192.168.1.80')
rosinit;

tag_sub = rossubscriber('/camera/tag_detections');
camera_info_sub = rossubscriber('/left_cam/camera_info');
color_point_sub = rossubscriber('/color_point');

fprintf('Done.\n');  
%% struct declare
num = struct('id',[],'tag_detection',[]);
color = struct('id','tag_detection');

%% main code
camera_info_data = receive(camera_info_sub, 10);
fx = camera_info_data.K(1);
fy = camera_info_data.K(5);    
color = ['ro'; 'yo'; 'ko'; 'bo']; %¡@set colors which represent each tags
color_arrow = ['r'; 'y'; 'k'; 'b']; %¡@set colors which represent each tags
tag0_check = 0; % existence checking variable of tag0 (0:inexist, 1:exist)

while 1;    
    fprintf('----------------------------------------\n'); 
    % receive topics
    tag_data = receive(tag_sub, 10);
    color_point_data = receive(color_point_sub, 10);
    
    dections = tag_data.Detections;
    num.tag_detection = size(dections,1)
    
    % avoid the interruption caused by zero tag dection
    if num.tag_detection == 0 
        continue;
    end
    
    % extract massages
    for i = 1:num.tag_detection
    id = dections(i).Id;
    num.id = id + 1;
    
    % extract pixel coordinate of apriltags
    pixcoord_tag{num.id}(1) = round(dections(i).Pixcoord.X);
    pixcoord_tag{num.id}(2) = camera_info_data.Height - round(dections(i).Pixcoord.Y); % set the pixel coordinate of tag0 and make the lower-left corner to be (0,0)
    
    % extract pose of apriltags
    pos{num.id}(1) = dections(i).Pose.Pose.Position.X;
    pos{num.id}(2) = -dections(i).Pose.Pose.Position.Y;
    pos{num.id}(3) = dections(i).Pose.Pose.Position.Z;
    
    quat{num.id}(1) = dections(i).Pose.Pose.Orientation.X;
    quat{num.id}(2) = dections(i).Pose.Pose.Orientation.Y;
    quat{num.id}(3) = dections(i).Pose.Pose.Orientation.Z;
    quat{num.id}(4) = dections(i).Pose.Pose.Orientation.W;
    
    % calculate Euler angles of tags
    eulZYX{num.id} = -quat2eul(quat{num.id});
    
    
    figure(1)
    % plot tags
    plot(pos{num.id}(1),pos{num.id}(2),color(num.id,:)); % for 2-d
    hold on 
    
    % plot orientation of tags
    u = 0.05*cos(eulZYX{num.id}(3));
    v = 0.05*sin(eulZYX{num.id}(3));  
    h = quiver(pos{num.id}(1),pos{num.id}(2),u,v,color_arrow(num.id,:),'Linewidth',3);
    set(h,'maxheadsize',1);
    hold on
    
    % plot color point
    if color_point_data.Point.Z == 1 % Z is the detection state (0:inexist 1:exist)
        % extract pixel coordinate of color point
        px = color_point_data.Point.X;
        py = double(camera_info_data.Height) - color_point_data.Point.Y;
    else
        px = [];
        py = [];
    end
    
    % plot color point
    if i == num.tag_detection 
        % check if tag0 exist or not
        for j = 1:num.tag_detection
            tag0_check = tag0_check || (dections(j).Id == 0);
        end
        
        if tag0_check == 1 % tag0 exist
            % mesaure anverage distance between tags and camera
            Pz_temp = 0;
            for k = 1:num.tag_detection
                id = dections(k).Id;
                num.id = id + 1;
                Pz_temp = Pz_temp + pos{num.id}(3);
            end
            Pz = Pz_temp/num.tag_detection;
            % calculate the position of color point
            Px = Pz*(px - pixcoord_tag{1}(1))/fx + pos{1}(1);
            Py = Pz*(py - pixcoord_tag{1}(2))/fy + pos{1}(2);
            
            plot(Px,Py,'g.','MarkerSize',20);
            tag0_check = 0; % reset tag0_check
        end
    end
    
    grid on
    axis([-0.3 0.3 -0.3 0.3]);
    axis square
    xlabel('x');ylabel('y');zlabel('z');
    hold on
    end
    hold off
end
%% close hardware
% Disconnect ROS master
rosshutdown;
clear all ; close all ; clc;

