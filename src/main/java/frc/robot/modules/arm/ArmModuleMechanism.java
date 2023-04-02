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
    private final double ARM_LENGTH_METERS = Units.inchesToMeters(21);
    private final double GRIPPER_CLOSE_ANGLE = 20.0;
    private final double GRIPPER_OPEN_ANGLE = 65.0;
    private final double SUPPORT1_LENGTH_METERS = Units.inchesToMeters(5);
    private final double SUPPORT2_LENGTH_METERS = Units.inchesToMeters(6);
    private final double TOWER_LENGTH_METERS = Units.inchesToMeters(30);

    private final MechanismRoot2d pivot;
    private final MechanismLigament2d tower;
    private final MechanismLigament2d support1;
    private final MechanismLigament2d support2;
    private final MechanismLigament2d arm;
    private final MechanismLigament2d gripperRightArm;
    private final MechanismLigament2d gripperLeftArm;

    /**
     * 
     */
    public ArmModuleMechanism() {
        this.pivot = MechanismConstants.CANVAS.getRoot("ARM PIVOT", Units.inchesToMeters(20), Units.inchesToMeters(37));
        this.tower = new MechanismLigament2d("ARM TOWER", TOWER_LENGTH_METERS, -70, 6, new Color8Bit(Color.kYellow));
        this.support1 = new MechanismLigament2d("TOWER SUPPORT 1 ", SUPPORT1_LENGTH_METERS, 0, 6, new Color8Bit(Color.kYellow));
        this.support2 = new MechanismLigament2d("TOWER SUPPORT 2", SUPPORT2_LENGTH_METERS, -60, 6, new Color8Bit(Color.kYellow));
        this.arm = new MechanismLigament2d("ARM", ARM_LENGTH_METERS, 0.0, 6, new Color8Bit(Color.kSilver));

        this.pivot.append(this.tower);
        this.tower.append(this.support1);
        this.tower.append(this.support2);
        this.pivot.append(this.arm);
        
        this.gripperRightArm = this.arm.append(new MechanismLigament2d("GRIPPER RIGHT ARM", Units.inchesToMeters(5), GRIPPER_OPEN_ANGLE, 6, new Color8Bit(Color.kBrown)));
        this.gripperRightArm.append(new MechanismLigament2d("GRIPPER RIGHT HAND", Units.inchesToMeters(6), -35, 6, new Color8Bit(Color.kGreen)));

        this.gripperLeftArm = this.arm.append(new MechanismLigament2d("GRIPPER LEFT ARM", Units.inchesToMeters(5), -GRIPPER_OPEN_ANGLE, 6, new Color8Bit(Color.kBrown)));
        this.gripperLeftArm.append(new MechanismLigament2d("GRIPPER LEFT HAND", Units.inchesToMeters(6), 35, 6, new Color8Bit(Color.kGreen)));
    }

    /**
     * 
     */
    public void close() {
        this.gripperRightArm.setAngle(GRIPPER_CLOSE_ANGLE);
        this.gripperLeftArm.setAngle(-GRIPPER_CLOSE_ANGLE);
    }

    /**
     * 
     */
    public void open() {
        this.gripperRightArm.setAngle(GRIPPER_OPEN_ANGLE);
        this.gripperLeftArm.setAngle(-GRIPPER_OPEN_ANGLE);
    }
    
    /**
     * 
     */
    public void setAngle(double arm_angle_radians) {
        this.arm.setAngle(Units.radiansToDegrees(arm_angle_radians));
    }
}
