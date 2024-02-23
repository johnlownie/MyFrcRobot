package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.modules.vision.AprilTagShootData;
import frc.robot.modules.vision.VisionModule;

/**
 * 
 */
public class VisionSubsystem extends SubsystemBase {
    /* Modules */
    private final VisionModule visionModule;

    // The tag data needed to move into position, set arm, and shoot
    private final List<AprilTagShootData> aprilTagShootDataList;

    /**
     * 
     */
    public VisionSubsystem(VisionModule visionModule) {
        this.visionModule = visionModule;
        this.aprilTagShootDataList = loadAprilTagShootData();
    }

    @Override
    public void periodic() {
        this.visionModule.process();
    }

    /**
     * 
     */
    public EstimatedRobotPose getBestLatestEstimatedPose() {
        return this.visionModule.getBestLatestEstimatedPose();
    }
 
    /**
     * 
     */
    public PhotonTrackedTarget getBestTarget(boolean fromFrontCamera) {
        return this.visionModule.getBestTarget(fromFrontCamera);
    }

    /**
     * 
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        return this.visionModule.getEstimationStdDevs(estimatedPose);
    }

    /**
     * 
     */
    public ArmSubsystem.Action getMoveTo(int id) {
        AprilTagShootData aprilTagShootData = this.aprilTagShootDataList.stream()
            .filter(shootData -> shootData.getId() == id)
            .findFirst()
            .get();

        return aprilTagShootData == null ? ArmSubsystem.Action.IDLE : aprilTagShootData.getMoveTo();
    }

    /**
     * 
     */
    public ShooterSubsystem.Action getShootTo(int id) {
        AprilTagShootData aprilTagShootData = this.aprilTagShootDataList.stream()
            .filter(shootData -> shootData.getId() == id)
            .findFirst()
            .get();

        return aprilTagShootData == null ? ShooterSubsystem.Action.IDLE : aprilTagShootData.getShootTo();
    }

    /**
     * 
     */
    public double getTagOffset(int id) {
        AprilTagShootData aprilTagShootData = this.aprilTagShootDataList.stream()
            .filter(shootData -> shootData.getId() == id)
            .findFirst()
            .get();

        return aprilTagShootData == null ? -1 : aprilTagShootData.getOffset();
    }

    /**
     * 
     */
    private List<AprilTagShootData> loadAprilTagShootData() {
        List<AprilTagShootData> tagShootList = new ArrayList<AprilTagShootData>();

        for (AprilTag aprilTag : FieldConstants.TAG_FIELD_LAYOUT.getTags()) {
            AprilTagShootData aprilTagShootData = new AprilTagShootData(aprilTag.ID, aprilTag.pose);
            tagShootList.add(aprilTagShootData);
        }

        return tagShootList;
    }

    /**
     * Only used in simulation
     */
    public void resetFieldPosition(Pose2d pose2d) {
        this.visionModule.resetFieldPosition(pose2d);
    }

    /**
     * Only used in simulation
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.visionModule.setPoseSupplier(poseSupplier);
    }
}
