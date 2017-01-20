package apriltags_ros;

public interface AprilTagDetectionArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apriltags_ros/AprilTagDetectionArray";
  static final java.lang.String _DEFINITION = "apriltags/AprilTagDetection[] detections";
  java.util.List<apriltags.AprilTagDetection> getDetections();
  void setDetections(java.util.List<apriltags.AprilTagDetection> value);
}
