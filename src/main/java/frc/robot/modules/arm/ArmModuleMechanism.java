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
    private final double ARM_LENGTH_METERS = Units.inchesToMeters(20);
    private final double GRIPPER_CLOSE_ANGLE = 20.0;
    private final double GRIPPER_OPEN_ANGLE = 45.0;
    private final double TOWER_HEIGHT_METERS = Units.inchesToMeters(30);

    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d arm;
    private final MechanismLigament2d gripperRightArm;
    private final MechanismLigament2d gripperLeftArm;

    /**
     * 
     */
    public ArmModuleMechanism() {
        this.armPivot = MechanismConstants.CANVAS.getRoot("ARM PIVOT", MechanismConstants.CANVAS_SIZE_METERS / 2, MechanismConstants.CANVAS_SIZE_METERS / 2);

        this.armPivot.append(new MechanismLigament2d("ARM TOWER", TOWER_HEIGHT_METERS, -90, 6, new Color8Bit(Color.kBlue)));
        this.arm = this.armPivot.append(new MechanismLigament2d("ARM", ARM_LENGTH_METERS, 0.0, 6, new Color8Bit(Color.kYellow)));
        
        this.gripperRightArm = this.arm.append(new MechanismLigament2d("GRIPPER RIGHT ARM", Units.inchesToMeters(4), GRIPPER_OPEN_ANGLE, 6, new Color8Bit(Color.kBrown)));
        this.gripperLeftArm = this.arm.append(new MechanismLigament2d("GRIPPER LEFT ARM", Units.inchesToMeters(4), -GRIPPER_OPEN_ANGLE, 6, new Color8Bit(Color.kBrown)));

        this.gripperRightArm.append(new MechanismLigament2d("GRIPPER RIGHT HAND", Units.inchesToMeters(4), -35, 6, new Color8Bit(Color.kGreen)));
        this.gripperLeftArm.append(new MechanismLigament2d("GRIPPER LEFT HAND", Units.inchesToMeters(4), 35, 6, new Color8Bit(Color.kGreen)));
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
