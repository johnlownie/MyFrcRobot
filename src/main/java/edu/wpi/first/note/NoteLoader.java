package edu.wpi.first.note;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * 
 */
public class NoteLoader {
    /**
     * 
     */
    public static List<VisionTargetSim> getVisionTargets() {
        List<VisionTargetSim> visionTargetSims = new ArrayList<VisionTargetSim>();
        VisionTargetSim visionTargetSim;

        // Rectangular target
        // TargetModel targetModel = new TargetModel(0.36, 0.05);

        // Spherical target
        TargetModel targetModel = new TargetModel(0.05);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(13.638911, 4.1021), new Rotation2d())), targetModel, 21);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(13.638911, 5.5499), new Rotation2d())), targetModel, 22);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(13.638911, 6.9977), new Rotation2d())), targetModel, 23);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 7.4549), new Rotation2d())), targetModel, 24);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 5.7785), new Rotation2d())), targetModel, 25);
        // visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 4.1021), new Rotation2d())), targetModel, 26);
        // visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 2.4257), new Rotation2d())), targetModel, 27);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 0.7493), new Rotation2d())), targetModel, 28);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(2.940431, 6.9977), new Rotation2d())), targetModel, 29);
        // visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(2.940431, 5.5499), new Rotation2d())), targetModel, 30);
        // visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(2.940431, 4.1021), new Rotation2d())), targetModel, 31);
        // visionTargetSims.add(visionTargetSim);

        return visionTargetSims;
    }
}
