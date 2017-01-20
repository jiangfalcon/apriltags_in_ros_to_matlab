classdef AprilTagDetectionArray < robotics.ros.Message
    %AprilTagDetectionArray MATLAB implementation of apriltags_ros/AprilTagDetectionArray
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2017 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'apriltags_ros/AprilTagDetectionArray' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '93c0a301ed9e6633dc34b8117d49ebd4' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        ApriltagsAprilTagDetectionClass = robotics.ros.msg.internal.MessageFactory.getClassForType('apriltags/AprilTagDetection') % Dispatch to MATLAB class for message type apriltags/AprilTagDetection
    end
    
    properties (Dependent)
        Detections
    end
    
    properties (Access = protected)
        Cache = struct('Detections', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Detections'} % List of non-constant message properties
        ROSPropertyList = {'detections'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = AprilTagDetectionArray(msg)
            %AprilTagDetectionArray Construct the message object AprilTagDetectionArray
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function detections = get.Detections(obj)
            %get.Detections Get the value for property Detections
            if isempty(obj.Cache.Detections)
                javaArray = obj.JavaMessage.getDetections;
                array = obj.readJavaArray(javaArray, obj.ApriltagsAprilTagDetectionClass);
                obj.Cache.Detections = feval(obj.ApriltagsAprilTagDetectionClass, array);
            end
            detections = obj.Cache.Detections;
        end
        
        function set.Detections(obj, detections)
            %set.Detections Set the value for property Detections
            if ~isvector(detections) && isempty(detections)
                % Allow empty [] input
                detections = feval([obj.ApriltagsAprilTagDetectionClass '.empty'], 0, 1);
            end
            
            validateattributes(detections, {obj.ApriltagsAprilTagDetectionClass}, {'vector'}, 'AprilTagDetectionArray', 'Detections');
            
            javaArray = obj.JavaMessage.getDetections;
            array = obj.writeJavaArray(detections, javaArray, obj.ApriltagsAprilTagDetectionClass);
            obj.JavaMessage.setDetections(array);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Detections)
                obj.Cache.Detections = [];
                obj.Cache.Detections = obj.Detections;
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Detections = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Recursively copy compound properties
            cpObj.Detections = copy(obj.Detections);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            DetectionsCell = arrayfun(@(x) feval([obj.ApriltagsAprilTagDetectionClass '.loadobj'], x), strObj.Detections, 'UniformOutput', false);
            obj.Detections = vertcat(DetectionsCell{:});
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Detections = arrayfun(@(x) saveobj(x), obj.Detections);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.apriltags_ros.AprilTagDetectionArray.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.apriltags_ros.AprilTagDetectionArray;
            obj.reload(strObj);
        end
    end
end