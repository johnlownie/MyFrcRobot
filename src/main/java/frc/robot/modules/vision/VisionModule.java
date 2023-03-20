package frc.robot.modules.vision;

import java.io.IOException;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 */
@AutoLog
abstract public class VisionModule implements Runnable, LoggableInputs {
    protected AprilTagFieldLayout aprilTagFieldLayout;
    protected PhotonCamera photonCamera;

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
    }

    abstract public EstimatedRobotPose grabLatestEstimatedPose();
    abstract public void setPoseSupplier(Supplier<Pose2d> poseSupplier);
    
    @Override
    public void run() {      
    }

    @Override
    public void fromLog(LogTable table) {
    }
    
    @Override
    public void toLog(LogTable table) {
    }

    /**
     * Getters and Setters
     */
    public PhotonCamera getCamera() { return this.photonCamera; }

    public void setCamera (PhotonCamera photonCamera) { this.photonCamera = photonCamera; }
}
