package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Timer;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.utils.Conversions;

/**
 * 
 */
public class PoseEstimatorSubsystem extends SubsystemBase {
    /* Subsystems */
    private final VisionSubsystem visionSubsystem;
    
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
 
    protected Notifier visionNotifier;

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d estimatedPoseWithoutGyro;
    private Pose2d previousPoseWithoutGyro;
    private double previousTimestamp = 0.0;

    private OriginPosition originPosition = OriginPosition.kRedAllianceWallRightSide;
    private final SwerveModulePosition[] defaultModulePositions = new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    private final SwerveModulePosition[] previousModulePositions = new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.9);

    /**
     * 
     */
    public PoseEstimatorSubsystem(Supplier<SwerveModulePosition[]> modulePositionSupplier, Supplier<Rotation2d> rotationSupplier, VisionSubsystem visionSubsystem) {
        this.modulePositionSupplier = modulePositionSupplier;
        this.rotationSupplier = rotationSupplier;
        this.visionSubsystem = visionSubsystem;
        this.visionSubsystem.setPoseSupplier(this::getCurrentPose);
        
        setSwerveDrivePoseEstimators();
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(
            new Pose2d(
                new Translation2d(FieldConstants.LENGTH_METERS, FieldConstants.WIDTH_METERS),
                new Rotation2d(Math.PI)));
    }

    /**
     * 
     */
    public Pose2d getCurrentPose() {
        return this.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * 
     */
    public SwerveModulePosition[] getModulePositions() {
        return this.modulePositionSupplier.get();
    }

    /**
     * 
     */
    public Rotation2d getRotation() {
        return isConnected() ? this.rotationSupplier.get() : this.estimatedPoseWithoutGyro.getRotation();
    }

    /**
     * 
     */
    private boolean isConnected() {
        Rotation2d rotation2d = this.rotationSupplier.get();

        return rotation2d != null;
    }

    @Override
    public void periodic() {
        double timeBetweenUpdates = Timer.getFPGATimestamp() - this.previousTimestamp;

        // if the gyro is not connected, use the swerve module positions to estimate the robot's rotation
        if (!isConnected()) {
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[this.previousModulePositions.length];
            SwerveModulePosition[] currentPositions = getModulePositions();
            for (int i = 0; i < moduleDeltas.length; i++) {
                SwerveModulePosition current = currentPositions[i];
                SwerveModulePosition previous = this.previousModulePositions[i];
                
                moduleDeltas[i] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
                previous.distanceMeters = current.distanceMeters;
            }
            
            Twist2d twist = DriveTrainConstants.SWERVE_DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
            this.estimatedPoseWithoutGyro = this.estimatedPoseWithoutGyro.exp(twist);
        }
    
        // Process the data from the vision module
        EstimatedRobotPose visionPose = this.visionSubsystem.getBestLatestEstimatedPose();
        if (visionPose != null) {
            Pose2d pose2d = visionPose.estimatedPose.toPose2d();
            Matrix<N3, N1> estimatedStdDevs = this.visionSubsystem.getEstimationStdDevs(pose2d);
            this.swerveDrivePoseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds, estimatedStdDevs);
            Logger.recordOutput("Subsystems/PoseEstimator/VisionPose", pose2d);
        }
        
        // Update pose estimator with drivetrain sensors
        this.swerveDrivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions());

        if (!isConnected()) {
            double velocityMetersPerSecond = Conversions.distanceBetween(estimatedPoseWithoutGyro, previousPoseWithoutGyro) / timeBetweenUpdates;
            this.previousPoseWithoutGyro = this.estimatedPoseWithoutGyro;
            this.previousTimestamp = Timer.getFPGATimestamp();

            Logger.recordOutput("Subsystems/PoseEstimator/EstimatedVelocityMPS", velocityMetersPerSecond);
        }

        // log poses, 3D geometry, and swerve module states, gyro offset
        Logger.recordOutput("Subsystems/PoseEstimator/Robot", getCurrentPose());
        Logger.recordOutput("Subsystems/PoseEstimator/RobotNoGyro", this.estimatedPoseWithoutGyro);
        Logger.recordOutput("Subsystems/PoseEstimator/PreviousRobotNoGyro", this.previousPoseWithoutGyro);
        Logger.recordOutput("Subsystems/PoseEstimator/Rotation", getRotation().getDegrees());
        Logger.recordOutput("Subsystems/PoseEstimator/3DFieldPose", new Pose3d(getCurrentPose()));
        
        Transform3d frontCameraView = new Transform3d(
            new Translation3d(getCurrentPose().getX(), getCurrentPose().getY(), Units.inchesToMeters(16)),
            new Rotation3d(0, Units.degreesToRadians(-40), getRotation().getRadians())
        );
        
        Transform3d rearCameraView = new Transform3d(
            new Translation3d(getCurrentPose().getX(), getCurrentPose().getY(), Units.inchesToMeters(16)),
            new Rotation3d(0, Units.degreesToRadians(-40), getRotation().getRadians() + Math.PI)
        );
        
        Transform3d noteCameraView = new Transform3d(
            new Translation3d(getCurrentPose().getX() + RobotConstants.ROBOT_LENGTH / 2, getCurrentPose().getY(), Units.inchesToMeters(6)),
            new Rotation3d(0, Units.degreesToRadians(10), getRotation().getRadians())
        );

        Logger.recordOutput("Subsystems/PoseEstimator/FrontCameraView", frontCameraView);
        Logger.recordOutput("Subsystems/PoseEstimator/RearCameraView", rearCameraView);
        Logger.recordOutput("Subsystems/PoseEstimator/NoteCameraView", noteCameraView);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetPose(Pose2d pose2d) {
        setCurrentPose(pose2d);
        this.visionSubsystem.resetFieldPosition(pose2d);
    }

    /**
     * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
     * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
     * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
     * right). Zero degrees is away from the driver and increases in the CCW direction.
     *
     * @param state the specified PathPlanner state to which is set the odometry
     */
    // public void resetOdometry(PathPlannerState state) {
        // this.estimatedPoseWithoutGyro = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
        // this.swerveDrivePoseEstimator.resetPosition(
            // getRotation(), 
            // getModulePositions(), 
            // new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation)
        // );
    // }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance, Pose2d alliancePose) {
        // AprilTagFieldLayout fieldTags = this.photonPoseEstimator.getFieldTags();
        boolean allianceChanged = false;

        switch (alliance) {
        case Blue:
            // fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
            this.originPosition = OriginPosition.kBlueAllianceWallRightSide;
            break;

        case Red:
            // fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
            this.originPosition = OriginPosition.kRedAllianceWallRightSide;
            break;

        default:
        }

        setCurrentPose(alliancePose);

        // The alliance changed, which changes the coordinate system.
        // Since a tag may have been seen and the tags are all relative to the coordinate system, the estimated pose
        // needs to be transformed to the new coordinate system.
        if (allianceChanged) {
            Pose2d newPose = flipAlliance(swerveDrivePoseEstimator.getEstimatedPosition());
            setCurrentPose(newPose);
        }
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        this.estimatedPoseWithoutGyro = newPose;
        this.swerveDrivePoseEstimator.resetPosition(getRotation(), getModulePositions(), newPose);
    }
    
    /**
     * 
     */
    private void setSwerveDrivePoseEstimators() {
        this.estimatedPoseWithoutGyro = new Pose2d();

        this.swerveDrivePoseEstimator =  new SwerveDrivePoseEstimator(
            DriveTrainConstants.SWERVE_DRIVE_KINEMATICS,
            new Rotation2d(),
            this.defaultModulePositions,
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs
        );

        // this.swerveDrivePoseEstimator =  new SwerveDrivePoseEstimator(
        //     DriveTrainConstants.SWERVE_DRIVE_KINEMATICS,
        //     new Rotation2d(),
        //     this.defaultModulePositions,
        //     new Pose2d()
        // );
    }
}
