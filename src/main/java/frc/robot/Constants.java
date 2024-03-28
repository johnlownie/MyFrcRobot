package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.lib.camera.Camera;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;


public final class Constants {
    /**
     * 
     */
    public static final class RobotConstants {
        public static final double ROBOT_LENGTH = Units.inchesToMeters(38.0);
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

        /* Modes to match locations to drive to */
        public static enum DriveModeType {
            AMP, SPEAKER
        }
        
    }

    /**
     * TODO: Starting Angle can vary for intake position and horizontal position in simulation
     */
    public static class ArmConstants {
        public static final int STARTING_ANGLE = -33;
        public static final int ANGLE_AMP = STARTING_ANGLE + 144;
        public static final int ANGLE_CLIMB  = STARTING_ANGLE + 90;
        public static final int ANGLE_INTAKE = STARTING_ANGLE + 0;
        public static final int ANGLE_PODIUM = STARTING_ANGLE + 21;
        public static final int ANGLE_SPEAKER = STARTING_ANGLE + 5;
        public static final int ANGLE_STAGE = STARTING_ANGLE + 33;
    }

    /**
     * 
     */
    public static final class DriveTrainConstants {
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24);
        public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.0);
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-
    
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d( WHEEL_BASE_METERS / 2.0,  TRACK_WIDTH_METERS / 2.0), //Front Left
            new Translation2d( WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0), //Front Right
    
            new Translation2d(-WHEEL_BASE_METERS / 2.0,  TRACK_WIDTH_METERS / 2.0), // Rear Left
            new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0)  // Rear Right
        );
    }

    /**
     * 
     */
    public static class FieldConstants {
        // Page 4 & 5 of Layout & Marking Diagram manual (https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf) 
        public static final double LENGTH_METERS = Units.inchesToMeters(652.73);
        public static final double WIDTH_METERS = Units.inchesToMeters(323.0);

        // Start center of robot 1.0 meters from wall)
        public static final double POSE_X = Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2);
        public static final double[] POSE_Y = { 
            Units.inchesToMeters(279.13),
            Units.inchesToMeters(218.42),
            Units.inchesToMeters(62.64)
        };

        // public static final Pose2d[] ALLIANCE_POSES = new Pose2d[] {
        //     new Pose2d(POSE_X, POSE_Y[0], Rotation2d.fromDegrees(0)),
        //     new Pose2d(POSE_X, POSE_Y[1], Rotation2d.fromDegrees(0)),
        //     new Pose2d(POSE_X, POSE_Y[2], Rotation2d.fromDegrees(0))
        // };

        // public static final Pose2d[] SPEAKER_POSES = new Pose2d[] {
        //     new Pose2d(Units.inchesToMeters(36.17 / 2) + (RobotConstants.ROBOT_LENGTH / 4), Units.inchesToMeters(218.42 + 31.3) + (RobotConstants.ROBOT_LENGTH / 4), Rotation2d.fromDegrees(60.0)),
        //     new Pose2d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
        //     new Pose2d(Units.inchesToMeters(36.17 / 2) + (RobotConstants.ROBOT_LENGTH / 4), Units.inchesToMeters(218.42 - 31.3) - (RobotConstants.ROBOT_LENGTH / 4), Rotation2d.fromDegrees(-60.0))
        // };

        public static final Pose2d AUTONOMOUS_SHOOTING_POSE = new Pose2d(3.90, 5.54, Rotation2d.fromDegrees(0.0));

        public static final Pose2d[] SPEAKER_POSES = new Pose2d[] {
            new Pose2d(0.73, 6.76, Rotation2d.fromDegrees(60.0)),
            new Pose2d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
            new Pose2d(0.73, 4.29, Rotation2d.fromDegrees(-60.0))
        };

        public static final Pose2d[] TUNING_POSES = new Pose2d[] {
            new Pose2d(1.00, 7.00, Rotation2d.fromDegrees(0.0)),
            new Pose2d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
            new Pose2d(0.73, 4.29, Rotation2d.fromDegrees(-60.0))
        };

        // relative to rear camera
        public static final Translation3d[] SPEAKER_POSE_TRANSLATIONS = new Translation3d[] {
            new Translation3d(Units.inchesToMeters(36.17 / 2), Units.inchesToMeters(-31.3) - (RobotConstants.ROBOT_LENGTH / 2), 0.0),
            new Translation3d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2), 0.0, 0.0),
            new Translation3d(Units.inchesToMeters(36.17 / 2), Units.inchesToMeters(31.3) + (RobotConstants.ROBOT_LENGTH / 2), 0.0)
        };

        // relative to rear camera
        public static final Rotation3d[] SPEAKER_POSE_ROTATIONS = new Rotation3d[] {
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(-60)),
            new Rotation3d(0.0, 0.0, 0.0),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(60))
        };

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
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
    public static final class PIDConstants {
        // PID constants for arm module
        public static final double ARM_MODULE_KP = 15.0;
        public static final double ARM_MODULE_KI = 0.0;
        public static final double ARM_MODULE_KD = 0.0;

        // PID constants for shooter module
        public static final double SHOOTER_MODULE_KP = 15.0;
        public static final double SHOOTER_MODULE_KI = 0.0002;
        public static final double SHOOTER_MODULE_KD = 0.0;
        public static final double SHOOTER_MODULE_KF = 000172;
        
        // PID constants for swerve modules
        public static final double SWERVE_MODULE_DRIVE_KP = 0.0;
        public static final double SWERVE_MODULE_DRIVE_KI = 0.0;
        public static final double SWERVE_MODULE_DRIVE_KD = 0.0;
        
        public static final double SWERVE_MODULE_TURN_KP = 0.0;
        public static final double SWERVE_MODULE_TURN_KI = 0.0;
        public static final double SWERVE_MODULE_TURN_KD = 0.0;
        
        // PID constants for autonomous/pathplanner mode
        public static final double SWERVE_DRIVE_X_KP = 0.0;
        public static final double SWERVE_DRIVE_X_KI = 0.0;
        public static final double SWERVE_DRIVE_X_KD = 0.0;

        public static final double SWERVE_DRIVE_Y_KP = 0.0;
        public static final double SWERVE_DRIVE_Y_KI = 0.0;
        public static final double SWERVE_DRIVE_Y_KD = 0.0;

        public static final double SWERVE_DRIVE_OMEGA_KP = 0.0;
        public static final double SWERVE_DRIVE_OMEGA_KI = 0.0;
        public static final double SWERVE_DRIVE_OMEGA_KD = 0.0;
        
        // PID constants for simulated arm module
        public static final double SIM_ARM_MODULE_KP = 15.0;
        public static final double SIM_ARM_MODULE_KI = 0.0;
        public static final double SIM_ARM_MODULE_KD = 0.0;

        // PID constants for simulated swerve modules
        public static final double SIM_SWERVE_MODULE_DRIVE_KP = 0.2;
        public static final double SIM_SWERVE_MODULE_DRIVE_KI = 0.0;
        public static final double SIM_SWERVE_MODULE_DRIVE_KD = 0.0;
        
        public static final double SIM_SWERVE_MODULE_TURN_KP = 1.0;
        public static final double SIM_SWERVE_MODULE_TURN_KI = 0.0;
        public static final double SIM_SWERVE_MODULE_TURN_KD = 0.0;
        
        // PID constants for simulated autonomous/pathplanner mode
        public static final double SIM_SWERVE_DRIVE_X_KP = 0.2;
        public static final double SIM_SWERVE_DRIVE_X_KI = 0.0;
        public static final double SIM_SWERVE_DRIVE_X_KD = 0.0;

        public static final double SIM_SWERVE_DRIVE_Y_KP = 0.2;
        public static final double SIM_SWERVE_DRIVE_Y_KI = 0.0;
        public static final double SIM_SWERVE_DRIVE_Y_KD = 0.0;

        public static final double SIM_SWERVE_DRIVE_OMEGA_KP = 0.0;
        public static final double SIM_SWERVE_DRIVE_OMEGA_KI = 0.0;
        public static final double SIM_SWERVE_DRIVE_OMEGA_KD = 0.0;

        /**
         * 
         */
        public static final double[] getDriveXPIDs() {
            return Robot.isReal() 
                ? new double[] { SWERVE_DRIVE_X_KP, SWERVE_DRIVE_X_KI, SWERVE_DRIVE_X_KD }
                : new double[] { SIM_SWERVE_DRIVE_X_KP, SIM_SWERVE_DRIVE_X_KI, SIM_SWERVE_DRIVE_X_KD };
        }

        /**
         * 
         */
        public static final double[] getDriveYPIDs() {
            return Robot.isReal() 
                ? new double[] { SWERVE_DRIVE_Y_KP, SWERVE_DRIVE_Y_KI, SWERVE_DRIVE_Y_KD }
                : new double[] { SIM_SWERVE_DRIVE_Y_KP, SIM_SWERVE_DRIVE_Y_KI, SIM_SWERVE_DRIVE_Y_KD };
        }
        /**
         * 
         */
        public static final double[] getDriveOmegaPIDs() {
            return Robot.isReal() 
                ? new double[] { SWERVE_DRIVE_OMEGA_KP, SWERVE_DRIVE_OMEGA_KI, SWERVE_DRIVE_OMEGA_KD }
                : new double[] { SIM_SWERVE_DRIVE_OMEGA_KP, SIM_SWERVE_DRIVE_OMEGA_KI, SIM_SWERVE_DRIVE_OMEGA_KD };
        }
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
            public static final int DRIVE_MOTOR_ID = 41;
            public static final int ANGLE_MOTOR_ID = 42;
            public static final int CANCODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(154.384);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 1;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-140.9765625 + 180);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 31;
            public static final int ANGLE_MOTOR_ID = 32;
            public static final int CANCODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-68.7304);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 21;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CANCODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(158.642578125 + 180);
        }
    }

    /**
     * 
     */
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

        public static final double SPEED_MODIFIER_ONE_HUNDRED = 1.00;
        public static final double SPEED_MODIFIER_THIRTY = 0.30;

        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4.5, 4);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4.5, 4);
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(10, 10);
    
        public static final ProfiledPIDController xController = new ProfiledPIDController(PIDConstants.SWERVE_DRIVE_X_KP, PIDConstants.SWERVE_DRIVE_X_KI, PIDConstants.SWERVE_DRIVE_X_KD, X_CONSTRAINTS);
        public static final ProfiledPIDController yController = new ProfiledPIDController(PIDConstants.SWERVE_DRIVE_Y_KP, PIDConstants.SWERVE_DRIVE_Y_KI, PIDConstants.SWERVE_DRIVE_Y_KD, Y_CONSTRAINTS);
        public static final ProfiledPIDController omegaController = new ProfiledPIDController(PIDConstants.SWERVE_DRIVE_OMEGA_KP, PIDConstants.SWERVE_DRIVE_OMEGA_KI, PIDConstants.SWERVE_DRIVE_OMEGA_KD, OMEGA_CONSTRAINTS);
    }

    /**
     * 
     */
    public static final class VisionConstants {
        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final Camera FRONT_CAMERA = new Camera("Arducam_OV9281-2",
            Camera.Type.APRILTAG, 0,
            Units.inchesToMeters(-10.0), Units.inchesToMeters(-6.25), Units.inchesToMeters(23.75),
            0.0, Units.degreesToRadians(24), 0.0
        );
        public static final Camera REAR_CAMERA = new Camera("Arducam_OV9281-1",
            Camera.Type.APRILTAG, 0,
            Units.inchesToMeters(-12.0), Units.inchesToMeters(-6.25), Units.inchesToMeters(23.75),
            0.0, Units.degreesToRadians(-35), Math.PI
        );
        public static final Camera NOTE_CAMERA = new Camera("WebCam",
            Camera.Type.COLOURED_SHAPE, 0,
            RobotConstants.ROBOT_LENGTH / 2, 0.0, Units.inchesToMeters(16),
            0.0, Units.degreesToRadians(10), 0.0
        );

        public static final List<Camera> CAMERAS = Arrays.asList(FRONT_CAMERA, REAR_CAMERA, NOTE_CAMERA);
        public static final String DATA_FROM_CAMERA = REAR_CAMERA.getName();
        
        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        // for simulation only
        public static final double DIAGONAL_FOV = 70;
        public static final int IMG_WIDTH = 800;
        public static final int IMG_HEIGHT = 600;
    }
 }
