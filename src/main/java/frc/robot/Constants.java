package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;

public final class Constants {
    /**
     * 
     */
    public static final class RobotConstants {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final boolean TUNING_MODE = false;
        
        // FIXME: update for various robots
        public enum Mode { REAL, REPLAY, SIM }
        public enum RobotType { ROBOT_2023_MK4I, ROBOT_DEFAULT, ROBOT_SIMBOT }

        private static final Alert invalidRobotAlert = new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);
        
        private static final RobotType ROBOT = RobotType.ROBOT_SIMBOT;

        // FIXME: update for various robots
        public static Mode getMode() {
            switch (getRobot()) {
                case ROBOT_DEFAULT:
                case ROBOT_2023_MK4I:
                    return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

                case ROBOT_SIMBOT:
                    return Mode.SIM;

                default:
                    return Mode.REAL;
            }
        }

        // FIXME: update for various robots
        public static RobotType getRobot() {
            if (RobotBase.isReal()) {
                if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
                    invalidRobotAlert.set(true);
                    return RobotType.ROBOT_DEFAULT;
                } else {
                    return ROBOT;
                }
            } else {
                return ROBOT;
            }
        }
    }

    /**
     * 
     */
    public static final class DriveTrainConstants {
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.0);
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.0);
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-
    
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d( WHEEL_BASE_METERS / 2.0,  TRACK_WIDTH_METERS / 2.0), //Front Left
            new Translation2d( WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0), //Front Right
    
            new Translation2d(-WHEEL_BASE_METERS / 2.0,  TRACK_WIDTH_METERS / 2.0), // Rear Left
            new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0)  // Rear Right
        );
    }

    public static class FieldConstants {
        // Page 4 & 5 of Layout & Marking Diagram manual (https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf) 
        public static final double LENGTH_METERS = Units.inchesToMeters(651.27);
        public static final double WIDTH_METERS = Units.inchesToMeters(315.1);

        public static final Pose2d[][] ALLIANCE_POSES = new Pose2d[][]{
            // Blue Alliance
            new Pose2d[]{
                new Pose2d(1.8, 4.43, Rotation2d.fromDegrees(0)),
                new Pose2d(1.8, 2.75, Rotation2d.fromDegrees(0)),
                new Pose2d(1.8, 0.51, Rotation2d.fromDegrees(0))
            },
            // Red Alliance
            new Pose2d[]{
                new Pose2d(14.68, 5.0, Rotation2d.fromDegrees(180)),
                new Pose2d(14.68, 6.0, Rotation2d.fromDegrees(180)),
                new Pose2d(14.68, 7.0, Rotation2d.fromDegrees(180))
            }
        };
    }

    /**
     * 
     */
    public static final class SwerveModuleConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * (1 / MK4I_L2.DRIVE_GEAR_RATIO) * MK4I_L2.WHEEL_DIAMETER_METERS * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DriveTrainConstants.TRACK_WIDTH_METERS / 2.0, DriveTrainConstants.TRACK_WIDTH_METERS / 2.0));

        public static final class MK4I_L2 {
            public static final double WHEEL_DIAMETER_METERS = 0.10033;
            public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
            public static final double DRIVE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
            public static final boolean DRIVE_MOTOR_INVERTED = true;
            public static final double ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
            public static final boolean ANGLE_MOTOR_INVERTED = true;

            private static final double ENCODER_RESEED_SECONDS = 10.0;
            private static final double ENCODER_RESEED_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
            private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
            public static final double TICKS_PER_ROTATION = 2048.0;
            public static final boolean CAN_CODER_INVERTED = false;
            public static final int CAN_TIMEOUT_MS = 250;
        }
    }

    public static final class TeleopConstants {
        public static final double DEADBAND = 0.1;
    
        public static final double X_RATE_LIMIT = 6.0;
        public static final double Y_RATE_LIMIT = 6.0;
        public static final double ROTATION_RATE_LIMIT = 5.0 * Math.PI;
    
        public static final double HEADING_MAX_VELOCITY = Math.PI * 4;
        public static final double HEADING_MAX_ACCELERATION = Math.PI * 16;
        
        public static final double HEADING_kP = 2.0;
        public static final double HEADING_kI = 0.0;
        public static final double HEADING_kD = 0.0;
    
        public static final double HEADING_TOLERANCE = Units.degreesToRadians(1.5);

        private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
    
        public static final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
        public static final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
        public static final ProfiledPIDController omegaController = new ProfiledPIDController(1.5, 0, 0, OMEGA_CONSTRAINTS);
    }

    public static class VisionConstants {
        public static final String REAR_CAMERA_NAME = "REAR_CAMERA";
           public static final Transform3d REAR_CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(0.0, 0.0, 0.5), // cam mounted center of robot, half meter up
                new Rotation3d(0, 0, 0));
        public static final Transform3d ROBOT_TO_REAR_CAMERA = REAR_CAMERA_TO_ROBOT.inverse();

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        // for simulation only
        public static final double DIAGONAL_FOV = 70;
        public static final int IMG_WIDTH = 1280;
        public static final int IMG_HEIGHT = 720;
    }
}
