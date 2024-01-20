package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.util.ProfiledPIDController;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;

public final class Constants {
    /**
     * 
     */
    public static final class RobotConstants {
        public static final double ROBOT_LENGTH = Units.inchesToMeters(34.50);
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final boolean TUNING_MODE = true;
        
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
        public static final double LENGTH_METERS = Units.inchesToMeters(651.25);
        public static final double WIDTH_METERS = Units.inchesToMeters(315.5);

        // AprilTag X + Rail distance = 40.45 + 14.28 (https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023FieldDrawings-CHARGEDUPSpecific.pdf)
        public static final double POSE_X = Units.inchesToMeters(40.45 + 14.28) + (RobotConstants.ROBOT_LENGTH / 2);
        public static final double[] POSE_Y = { Units.inchesToMeters(174.19), Units.inchesToMeters(108.19), Units.inchesToMeters(42.19) };
        public static final double POLE_STRAFE_DISTANCE = Units.inchesToMeters(22.0);
        public static final double STATION_STRAFE_DISTANCE = Units.inchesToMeters(22.0);

        public static final Pose2d[] ALLIANCE_POSES = new Pose2d[] {
            new Pose2d(POSE_X, POSE_Y[0], Rotation2d.fromDegrees(0)),
            new Pose2d(POSE_X, POSE_Y[1], Rotation2d.fromDegrees(0)),
            new Pose2d(POSE_X, POSE_Y[2], Rotation2d.fromDegrees(0))
        };

        public static final Pose2d[][] POLE_POSES = new Pose2d[][] {
            new Pose2d[] {
                new Pose2d(POSE_X, POSE_Y[0] + POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0)),
                new Pose2d(POSE_X, POSE_Y[0] - POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0))
            },
            new Pose2d[] {
                new Pose2d(POSE_X, POSE_Y[1] + POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0)),
                new Pose2d(POSE_X, POSE_Y[1] - POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0))
            },
            new Pose2d[] {
                new Pose2d(POSE_X, POSE_Y[2] + POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0)),
                new Pose2d(POSE_X, POSE_Y[2] - POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0))
            }
        };

        public static final Pose2d[] GAME_PIECE_POSES = new Pose2d[] {
            new Pose2d(Units.inchesToMeters(253.54), Units.inchesToMeters(180.71), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(261.0), Units.inchesToMeters(132.19), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(261.0), Units.inchesToMeters( 84.19), Rotation2d.fromDegrees(0)),
            new Pose2d(Units.inchesToMeters(261.0), Units.inchesToMeters( 36.19), Rotation2d.fromDegrees(0))
        };
        
        public static final Pose2d CHARGE_STATION_EDGE = new Pose2d(POSE_X + Units.inchesToMeters(119.25), POSE_Y[1] + POLE_STRAFE_DISTANCE, Rotation2d.fromDegrees(0));
    }

    /**
     * 
     */
    public static final class MechanismConstants {
        public static final double CANVAS_SIZE_METERS = Units.inchesToMeters(60);
        public static final Mechanism2d CANVAS = new Mechanism2d(CANVAS_SIZE_METERS, CANVAS_SIZE_METERS, new Color8Bit(Color.kLightGray));
    }

    /**
     * 
     */
    public static final class PneumaticConstants {
        public static final int HUB_ID = 50;

        public static final int DRAWER_OPEN_ID = 12;
        public static final int DRAWER_CLOSE_ID = 13;

        public static final int GRIPPER_OPEN_ID = 14;
        public static final int GRIPPER_CLOSE_ID = 15;
        
        public static final int[] IN_USE_CHANNELS = { 12, 13, 14, 15 };
    }

    /**
     * 
     */
    public static final class SwerveModuleConstants {
        public static final double ANGLE_CURRENT_LIMIT = 25;
        public static final double ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_TIME_THRESHOLD = 0.1;

        public static final double DRIVE_CURRENT_LIMIT = 35;
        public static final double DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_TIME_THRESHOLD = 0.1;

        public static final double STATOR_CURRENT_LIMIT = 20;

        public static final double CANCODER_UPDATE_FREQUENCY = 100;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10.0;

        public static final double TICKS_PER_ROTATION = 2048.0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 1;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 31;
            public static final int ANGLE_MOTOR_ID = 32;
            public static final int CANCODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 21;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CANCODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 41;
            public static final int ANGLE_MOTOR_ID = 42;
            public static final int CANCODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
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

        private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 3);
        private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 3);
        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
    
        public static final ProfiledPIDController xController = new ProfiledPIDController(6, 0, 0, X_CONSTRAINTS);
        public static final ProfiledPIDController yController = new ProfiledPIDController(6, 0, 0, Y_CONSTRAINTS);
        public static final ProfiledPIDController omegaController = new ProfiledPIDController(10.0, 0, 0, OMEGA_CONSTRAINTS);
    }

    /**
     * 
     */
    public static class VisionConstants {
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final String FRONT_CAMERA_NAME = "camera1";
        public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(0.0, 0.0, -0.5), // cam mounted center of robot, half meter up
            new Rotation3d(0, 0, 0));
        public static final Transform3d ROBOT_TO_FRONT_CAMERA = FRONT_CAMERA_TO_ROBOT.inverse();

        public static final String REAR_CAMERA_NAME = "REAR_CAMERA";
        public static final Transform3d REAR_CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(0.0, 0.0, -0.5), // cam mounted center of robot, half meter up
            new Rotation3d(0, 0, Math.PI));
        public static final Transform3d ROBOT_TO_REAR_CAMERA = REAR_CAMERA_TO_ROBOT.inverse();

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        // for simulation only
        public static final double DIAGONAL_FOV = 70;
        public static final int IMG_WIDTH = 800;
        public static final int IMG_HEIGHT = 600;
    }
 }
