package edu.wpi.first.note;

import java.util.ArrayList;
import java.util.List;

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

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(13.638911, 4.1021), new Rotation2d())), null, 1);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(13.638911, 5.5499), new Rotation2d())), null, 2);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(13.638911, 6.9977), new Rotation2d())), null, 3);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 7.4549), new Rotation2d())), null, 4);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 5.7785), new Rotation2d())), null, 5);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 4.1021), new Rotation2d())), null, 6);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 2.4257), new Rotation2d())), null, 7);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(8.289671, 0.7493), new Rotation2d())), null, 8);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(2.940431, 6.9977), new Rotation2d())), null, 9);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(2.940431, 5.5499), new Rotation2d())), null, 10);
        visionTargetSims.add(visionTargetSim);

        visionTargetSim = new VisionTargetSim(new Pose3d(new Pose2d(new Translation2d(2.940431, 4.1021), new Rotation2d())), null, 11);
        visionTargetSims.add(visionTargetSim);

        return visionTargetSims;
    }
}
