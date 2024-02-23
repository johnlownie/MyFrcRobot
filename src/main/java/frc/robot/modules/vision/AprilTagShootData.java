package frc.robot.modules.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * 
 */
public class AprilTagShootData {
    private int id;
    private Pose3d pose3d;
    private double offset;
    private ArmSubsystem.Action move_to;
    private ShooterSubsystem.Action shoot_to;

    /**
     * 
     */
    public AprilTagShootData(int id, Pose3d pose3d) {
        this.id = id;
        this.pose3d = pose3d;

        switch (id) {
            // SPEAKER
            case 4:
            case 7:
                this.offset = Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2);
                this.move_to = ArmSubsystem.Action.MOVE_TO_SPEAKER;
                this.shoot_to = ShooterSubsystem.Action.SHOOT_SPEAKER;
                break;

            // AMP
            case 5:
            case 6:
                this.offset = RobotConstants.ROBOT_LENGTH / 2;
                this.move_to = ArmSubsystem.Action.MOVE_TO_AMP;
                this.shoot_to = ShooterSubsystem.Action.SHOOT_AMP;
                break;

            // STAGE
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
                this.offset = 1.0;
                this.move_to = ArmSubsystem.Action.MOVE_TO_STAGE;
                this.shoot_to = ShooterSubsystem.Action.IDLE;
        }
    }

    /**
     * Getters and Setters
     */
    public int getId() { return this.id; }
    public ArmSubsystem.Action getMoveTo() { return this.move_to; }
    public ShooterSubsystem.Action getShootTo() { return this.shoot_to; }
    public double getOffset() { return this.offset; }
    public Pose3d getPose() { return this.pose3d; }
    
}
