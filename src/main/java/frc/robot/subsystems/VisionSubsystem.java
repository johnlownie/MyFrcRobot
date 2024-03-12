package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.camera.Camera;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.modules.vision.AprilTagShootData;
import frc.robot.modules.vision.VisionModule;
import frc.robot.modules.vision.VisionModuleSimulator;

/**
 * 
 */
public class VisionSubsystem extends SubsystemBase {
    /* Modules */
    private List<VisionModule> visionModules = new ArrayList<VisionModule>();

    // The tag data needed to move into position, set arm, and shoot
    private final List<AprilTagShootData> aprilTagShootDataList;

    /**
     * 
     */
    public VisionSubsystem(List<Camera> cameras) {
        for (Camera camera : cameras) {
            this.visionModules.add(Robot.isReal() ? new VisionModule(camera) : new VisionModuleSimulator(camera));
            // this.visionModules.add(new VisionModule(camera));
        }

        this.aprilTagShootDataList = loadAprilTagShootData();
    }

    @Override
    public void periodic() {
        for (VisionModule visionModule : this.visionModules) {
            visionModule.process();
        }
    }

    /**
     * Use AprilTag camera pose estimation
     */
    public List<EstimatedRobotPose> getBestLatestEstimatedPoses() {
        List<EstimatedRobotPose> poses = new ArrayList<EstimatedRobotPose>();

        for (VisionModule visionModule : this.visionModules) {
            if (visionModule.getCameraType() != Camera.Type.APRILTAG) continue;

            EstimatedRobotPose pose = visionModule.getBestLatestEstimatedPose();
            if (pose == null) continue;

            poses.add(pose);
        }

        return poses;
    }
 
    /**
     * Use target from the first Game Piece camera
     */
    public PhotonTrackedTarget getBestNoteTarget() {
        VisionModule visionModule = getVisionModuleByType(Camera.Type.COLOURED_SHAPE);

        return visionModule == null ? null : visionModule.getBestTarget();
    }

    /**
     * 
     */
    public PhotonTrackedTarget getBestTarget(String cameraName) {
        VisionModule visionModule = getVisionModuleByName(cameraName);

        return visionModule == null ? null : visionModule.getBestTarget();
    }
 
    /**
     * 
     */
    public Pose3d getBestTargetPose(String cameraName) {
        VisionModule visionModule = getVisionModuleByName(cameraName);

        if (visionModule == null) return null;

        PhotonTrackedTarget target = visionModule.getBestTarget();

        return target == null ? null : getTargetPose(target.getFiducialId());
    }

    /**
     * 
     */
    public double getCameraYaw(String cameraName) {
        VisionModule visionModule = getVisionModuleByName(cameraName);

        return visionModule == null ? null : visionModule.getCameraYaw();
    }

    /**
     * 
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        VisionModule visionModule = getVisionModuleByType(Camera.Type.APRILTAG);

        return visionModule == null ? null : visionModule.getEstimationStdDevs(estimatedPose);
    }

    /**
     * 
     */
    public Transform3d getRobotToCamera(String cameraName) {
        VisionModule visionModule = getVisionModuleByName(cameraName);

        return visionModule == null ? null : visionModule.getCameraRobotToCamera();
    }

    /**
     * 
     */
    public Pose3d getTargetPose(int id) {
        AprilTagShootData aprilTagShootData = this.aprilTagShootDataList.stream()
            .filter(shootData -> shootData.getId() == id)
            .findFirst()
            .get();

        return aprilTagShootData == null ? null : aprilTagShootData.getPose();
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
    public VisionModule getVisionModuleByName(String cameraName) {
        VisionModule visionModule = this.visionModules.stream()
            .filter(module -> module.getCameraName() == cameraName)
            .findFirst()
            .get();

        return visionModule;
    }

    /**
     * 
     */
    private VisionModule getVisionModuleByType(Camera.Type type) {
        VisionModule visionModule = this.visionModules.stream()
            .filter(module -> module.getCameraType() == type)
            .findFirst()
            .get();

        return visionModule;
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
        for (VisionModule visionModule : this.visionModules) {
            visionModule.resetFieldPosition(pose2d);
        }
    }
    
    /**
     * Only used in simulation
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        for (VisionModule visionModule : this.visionModules) {
            visionModule.setPoseSupplier(poseSupplier);
        }
    }
}
