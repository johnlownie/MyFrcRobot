package frc.robot.modules.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.MechanismConstants;

/**
 * 
 */
public class ArmModuleMechanism {
    private final double ARM_LENGTH_METERS = Units.inchesToMeters(16);
    private final double GRIPPER_CLOSE_ANGLE = 20.0;
    private final double GRIPPER_OPEN_ANGLE = 65.0;
    private final double ROBOT_BASE1_LENGTH_METERS = Units.inchesToMeters(4.5);
    private final double ROBOT_BASE2_LENGTH_METERS = Units.inchesToMeters(27.5);
    private final double SHOOTER_BASE1_LENGTH_METERS = Units.inchesToMeters(7.5);
    private final double SHOOTER_BASE2_LENGTH_METERS = Units.inchesToMeters(7.5);
    private final double SUPPORT_LENGTH_METERS = Units.inchesToMeters(19.63);
    private final double TOWER_LENGTH_METERS = Units.inchesToMeters(17);
    private final double LINE_WIDTH = 6;

    private final MechanismRoot2d pivot;
    private final MechanismLigament2d tower;
    private final MechanismLigament2d support;
    private final MechanismLigament2d robotBase1;
    private final MechanismLigament2d robotBase2;
    private final MechanismLigament2d arm;
    private final MechanismLigament2d shooterBase1;
    private final MechanismLigament2d shooterBase2;

    /**
     * 
     */
    public ArmModuleMechanism() {
        this.pivot = MechanismConstants.CANVAS.getRoot("ARM PIVOT", Units.inchesToMeters(20), TOWER_LENGTH_METERS + Units.inchesToMeters(3));
        this.tower = new MechanismLigament2d("ARM TOWER", TOWER_LENGTH_METERS, -90, LINE_WIDTH, new Color8Bit(Color.kRoyalBlue));
        this.support = new MechanismLigament2d("TOWER SUPPORT ", SUPPORT_LENGTH_METERS,-60, LINE_WIDTH, new Color8Bit(Color.kRoyalBlue));
        this.robotBase1 = new MechanismLigament2d("ROBOT BASE 1", ROBOT_BASE1_LENGTH_METERS, 270, LINE_WIDTH, new Color8Bit(Color.kRoyalBlue));
        this.robotBase2 = new MechanismLigament2d("ROBOT BASE 2", ROBOT_BASE2_LENGTH_METERS, 90, LINE_WIDTH, new Color8Bit(Color.kRoyalBlue));
        this.arm = new MechanismLigament2d("ARM", ARM_LENGTH_METERS, 0, LINE_WIDTH, new Color8Bit(Color.kBlue));
        this.shooterBase1 = new MechanismLigament2d("SHOOTER BASE 1", SHOOTER_BASE1_LENGTH_METERS, -45, LINE_WIDTH, new Color8Bit(Color.kRed));
        this.shooterBase2 = new MechanismLigament2d("SHOOTER BASE 2", SHOOTER_BASE2_LENGTH_METERS, 135, LINE_WIDTH, new Color8Bit(Color.kOrange));

        this.pivot.append(this.tower);
        this.pivot.append(this.support);
        this.tower.append(this.robotBase1);
        this.tower.append(this.robotBase2);
        this.pivot.append(this.arm);
        this.arm.append(this.shooterBase1);
        this.arm.append(this.shooterBase2);
        
        // this.gripperRightArm = this.arm.append(new MechanismLigament2d("GRIPPER RIGHT ARM", Units.inchesToMeters(5), GRIPPER_OPEN_ANGLE, 6, new Color8Bit(Color.kBrown)));
        // this.gripperRightArm.append(new MechanismLigament2d("GRIPPER RIGHT HAND", Units.inchesToMeters(6), -35, 6, new Color8Bit(Color.kGreen)));

        // this.gripperLeftArm = this.arm.append(new MechanismLigament2d("GRIPPER LEFT ARM", Units.inchesToMeters(5), -GRIPPER_OPEN_ANGLE, 6, new Color8Bit(Color.kBrown)));
        // this.gripperLeftArm.append(new MechanismLigament2d("GRIPPER LEFT HAND", Units.inchesToMeters(6), 35, 6, new Color8Bit(Color.kGreen)));
    }

    /**
     * 
     */
    public void setAngle(double arm_angle_radians) {
        this.arm.setAngle(Units.radiansToDegrees(arm_angle_radians));
    }
}
