package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  public static final double fieldBorderMargin = 0.2; // meters

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static class LimelightConfigs {
    public final String name;
    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    public final double forwardOffsetMeters; // meters
    public final double sideOffsetMeters; // meters
    public final double heightOffsetMeters; // meters
    public final double rollDegrees; // degrees
    public final double pitchDegrees; // degrees
    public final double yawDegrees; // degrees

    public LimelightConfigs(
        String name,
        double forwardOffsetMeters,
        double sideOffsetMeters,
        double heightOffsetMeters,
        double rollDegrees,
        double pitchDegrees,
        double yawDegrees) {
      this.name = name;
      this.forwardOffsetMeters = forwardOffsetMeters;
      this.sideOffsetMeters = sideOffsetMeters;
      this.heightOffsetMeters = heightOffsetMeters;
      this.rollDegrees = rollDegrees;
      this.pitchDegrees = pitchDegrees;
      this.yawDegrees = yawDegrees;
    }
  }

  public static final LimelightConfigs starboardFore =
      new LimelightConfigs("sfore", 0, 0, 0, 0, 0, 0);
  public static final LimelightConfigs starboardAft =
      new LimelightConfigs("saft", -0.308, 0.256, 0.202111, 0, 24.45, 207.785);
  public static final LimelightConfigs portFore =
      new LimelightConfigs("pfore", -0.247, -0.286, 0.228, 0, 24.5, 45);
  public static final LimelightConfigs portAft =
      new LimelightConfigs("paft", -0.301, -0.162, 0.23, 0, 28.1, 145);

  // what r these
  public static final double megatag1AmbiguityMinimum = 0.7; // TODO change
  public static final double megatag1DistanceMaximum = 0.3; // meter //TODO change

  // can't log rawDetection class so heres a reference so that i know what index of the double
  // corresponds ot what
  public static class rawDetectionRef {
    public static int classId = 0;
    public static int txnc = 1;
    public static int tync = 2;
    public static int ta = 3;
    public static int corner0_X = 4;
    public static int corner0_Y = 5;
    public static int corner1_X = 6;
    public static int corner1_Y = 7;
    public static int corner2_X = 8;
    public static int corner2_Y = 9;
    public static int corner3_X = 10;
    public static int corner3_Y = 11;
  }

  // TODO maybe make different pipelines for each camera? they might not all have the same
  // eventually.
  public static enum Pipelines {
    // TODO Add more pipelines, rename them to these names
    APRILTAG,
    OBJ_DETECTION;

    // TODO check to make sure that these are the correct order for the pipeline
    public static int getIndexFor(Pipelines pipeline) {
      switch (pipeline) {
        case APRILTAG:
          return 0;
        case OBJ_DETECTION:
          return 1;
        default:
          return 0;
      }
    }
  }
}
