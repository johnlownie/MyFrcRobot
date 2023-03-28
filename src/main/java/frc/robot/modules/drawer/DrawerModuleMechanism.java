package frc.robot.modules.drawer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.MechanismConstants;

/**
 * 
 */
public class DrawerModuleMechanism {
    private final MechanismRoot2d pivot;
    private final MechanismLigament2d arm;
    private final MechanismLigament2d bottom;

    private final double ARM_RETRACTED_LENGTH_METERS = Units.inchesToMeters(2);
    private final double ARM_EXTENDED_LENGTH_METERS = Units.inchesToMeters(16);
    private final double BOTTOM_LENGTH_METERS = Units.inchesToMeters(7);
    private final double SIDE_LENGTH_METERS = Units.inchesToMeters(8);

    /**
     * 
     */
    public DrawerModuleMechanism() {
        this.pivot = MechanismConstants.CANVAS.getRoot("DRAWER PIVOT", MechanismConstants.CANVAS_SIZE_METERS / 2, Units.inchesToMeters(6));
        this.arm = new MechanismLigament2d("DRAWER ARM", ARM_RETRACTED_LENGTH_METERS, 0, 6, new Color8Bit(Color.kRed));
        this.bottom = new MechanismLigament2d("DRAWER BOTTOM", BOTTOM_LENGTH_METERS, 0, 6, new Color8Bit(Color.kBlue));

        this.pivot.append(this.arm);
        this.arm.append(this.bottom);
        this.arm.append(new MechanismLigament2d("DRAWER SIDE 1", BOTTOM_LENGTH_METERS, 90, 6, new Color8Bit(Color.kBlue)));
        this.bottom.append(new MechanismLigament2d("DRAWER SIDE 2", SIDE_LENGTH_METERS, 50, 6, new Color8Bit(Color.kBlue)));
    }

    /**
     * 
     */
    public void extend() {
        this.arm.setLength(ARM_EXTENDED_LENGTH_METERS);
    }
    
    /**
     * 
     */
    public void retract() {
        this.arm.setLength(ARM_RETRACTED_LENGTH_METERS);
    }
}
