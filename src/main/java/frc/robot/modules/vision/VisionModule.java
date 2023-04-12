package frc.robot.modules.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * 
 */
@AutoLog
abstract public class VisionModule implements Runnable, LoggableInputs {
    protected AprilTagFieldLayout aprilTagFieldLayout;
    protected final PhotonPoseEstimator frontCameraPhotonPoseEstimator;
    protected final PhotonPoseEstimator rearCameraPhotonPoseEstimator;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose;

    protected PhotonCamera frontCamera;
    protected PhotonCamera rearCamera;
    protected Supplier<Pose2d> poseSupplier;

    /**
     * 
     */
    public VisionModule(AprilTagFields aprilTagFields) {
        try {
            this.aprilTagFieldLayout = aprilTagFields.loadAprilTagLayoutField();
            // aprilTagFieldLayout.setOrigin(this.originPosition);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTag Field Layout", e.getStackTrace());
        }

        this.frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);
        this.rearCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);

        this.frontCameraPhotonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, this.frontCamera, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearCameraPhotonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, this.rearCamera, VisionConstants.ROBOT_TO_REAR_CAMERA);

        this.atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    }

    /**
     * 
     */
    public PhotonTrackedTarget getBestTarget(boolean fromFrontCamera) {
        PhotonPipelineResult results = fromFrontCamera ? getFrontCameraResults() : getRearCameraResults();

        return results.hasTargets() ? results.getBestTarget() : null;
    }

    abstract protected PhotonPipelineResult getFrontCameraResults();
    abstract protected PhotonPipelineResult getRearCameraResults();
    abstract protected void processFrame(Pose2d pose);

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
     * new estimate that hasn't been returned before.
     * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     * @return latest estimated pose
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return this.atomicEstimatedRobotPose.getAndSet(null);
    }
    
    @Override
    public void run() {
        if (this.frontCameraPhotonPoseEstimator == null || this.rearCameraPhotonPoseEstimator == null || this.rearCamera == null || RobotState.isAutonomous()) {
            return;
        }
        
        processFrame(this.poseSupplier.get());

        processResults("FRONT_CAMERA", getFrontCameraResults(), this.frontCameraPhotonPoseEstimator);
        processResults("REAR_CAMERA", getRearCameraResults(), this.rearCameraPhotonPoseEstimator);
    }

    @Override
    public void fromLog(LogTable table) {
    }
    
    @Override
    public void toLog(LogTable table) {
    }

    /**
     * 
     */
    private void processResults(String camera, PhotonPipelineResult results, PhotonPoseEstimator poseEstimator) {
        Logger.getInstance().recordOutput("Subsystems/Vision/" + camera + "/hasTargets", results.hasTargets());
        Logger.getInstance().recordOutput("Subsystems/Vision/" + camera + "/TargetCount", results.hasTargets() ? results.getTargets().size() : 0);

        if (!results.hasTargets() || (results.targets.size() > 1 && results.targets.get(0).getPoseAmbiguity() > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
            return;
        }

        List<String> targetIds = new ArrayList<String>();
        for (PhotonTrackedTarget target : results.getTargets()) {
            targetIds.add("" + target.getFiducialId());
        }
        Logger.getInstance().recordOutput("Subsystems/Vision/" + camera + "/Target Ids", targetIds.toString());

        poseEstimator.update(results).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;

            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.WIDTH_METERS) {
              this.atomicEstimatedRobotPose.set(estimatedRobotPose);
            }

            EstimatedRobotPose pose = grabLatestEstimatedPose();
            Logger.getInstance().recordOutput("Subsystems/Vision/" + camera + "/Pose", pose != null ? pose.estimatedPose : new Pose3d());
        });

        if (results.hasTargets()) {
            PhotonTrackedTarget target = results.getBestTarget();
            
            Logger.getInstance().recordOutput("Subsystems/Vision/" + camera + "/Last Target Id", String.format("%d", target.getFiducialId()));
            Logger.getInstance().recordOutput("Subsystems/Vision/" + camera + "/Target Yaw", target.getYaw());
        }
    }

    /**
     * Getters and Setters
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) { this.poseSupplier = poseSupplier; }
}
