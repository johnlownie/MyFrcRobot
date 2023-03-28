package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.modules.vision.VisionModule;

/**
 * 
 */
public class PoseEstimatorSubsystem extends SubsystemBase {
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    private final VisionModule visionModule;
 
    protected Notifier visionNotifier;

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d estimatedPoseWithoutGyro;

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
    public PoseEstimatorSubsystem(Supplier<SwerveModulePosition[]> modulePositionSupplier, Supplier<Rotation2d> rotationSupplier, VisionModule visionModule) {
        this.modulePositionSupplier = modulePositionSupplier;
        this.rotationSupplier = rotationSupplier;
        this.visionModule = visionModule;
        this.visionModule.setPoseSupplier(this::getCurrentPose);
        
        setSwerveDrivePoseEstimators();
        startVisionThread();
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
        // if the gyro is not connected, use the swerve module positions to estimate the robot's rotation
        if (!isConnected()) {
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[this.previousModulePositions.length];
            for (int i = 0; i < moduleDeltas.length; i++) {
                SwerveModulePosition current = getModulePositions()[i];
                SwerveModulePosition previous = this.previousModulePositions[i];
                
                moduleDeltas[i] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters, current.angle);
                previous.distanceMeters = current.distanceMeters;
            }
            
            Twist2d twist = DriveTrainConstants.SWERVE_DRIVE_KINEMATICS.toTwist2d(moduleDeltas);
            this.estimatedPoseWithoutGyro = this.estimatedPoseWithoutGyro.exp(twist);
        }
        
        // Update pose estimator with drivetrain sensors
        this.swerveDrivePoseEstimator.update(getRotation(), getModulePositions());
    
        EstimatedRobotPose visionPose = this.visionModule.grabLatestEstimatedPose();
        if (visionPose != null) {
            Pose2d pose2d = visionPose.estimatedPose.toPose2d();
            
            this.swerveDrivePoseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
            Logger.getInstance().recordOutput("PoseEstimator/VisionPose", pose2d);
        }
        
        // log poses, 3D geometry, and swerve module states, gyro offset
        Logger.getInstance().recordOutput("PoseEstimator/Robot", getCurrentPose());
        Logger.getInstance().recordOutput("PoseEstimator/RobotNoGyro", this.estimatedPoseWithoutGyro);
        Logger.getInstance().recordOutput("PoseEstimator/Rotation", getRotation().getDegrees());
        Logger.getInstance().recordOutput("3DField", new Pose3d(getCurrentPose()));
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

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
            getRotation(),
            this.defaultModulePositions,
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs
        );
    }

    /**
     * 
     */
    private void startVisionThread() {
        // Start PhotonVision thread
        this.visionNotifier = new Notifier(this.visionModule);
        this.visionNotifier.setName("PhotonRunnable");
        this.visionNotifier.startPeriodic(0.02);
    }
}
