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
    private final MechanismRoot2d drawerPivot;
    private final MechanismLigament2d drawerArm;
    private final MechanismLigament2d drawerBottom;

    private final double DRAWER_ARM_RETRACTED_LENGTH_METERS = Units.inchesToMeters(10);
    private final double DRAWER_ARM_EXTENDED_LENGTH_METERS = Units.inchesToMeters(20);
    private final double DRAWER_BOTTOM_LENGTH_METERS = Units.inchesToMeters(5);
    private final double DRAWER_SIDE_LENGTH_METERS = Units.inchesToMeters(3);

    /**
     * 
     */
    public DrawerModuleMechanism() {
        this.drawerPivot = MechanismConstants.CANVAS.getRoot("DRAWER PIVOT", MechanismConstants.CANVAS_SIZE_METERS / 2, MechanismConstants.CANVAS_SIZE_METERS / 12);
        this.drawerArm = new MechanismLigament2d("DRAWER ARM", DRAWER_ARM_RETRACTED_LENGTH_METERS, 0, 6, new Color8Bit(Color.kRed));
        this.drawerBottom = new MechanismLigament2d("DRAWER BOTTOM", DRAWER_BOTTOM_LENGTH_METERS, 0, 6, new Color8Bit(Color.kBlue));

        this.drawerPivot.append(this.drawerArm);
        this.drawerArm.append(this.drawerBottom);
        this.drawerArm.append(new MechanismLigament2d("DRAWER SIDE", DRAWER_SIDE_LENGTH_METERS, 125, 6, new Color8Bit(Color.kGreen)));
        this.drawerBottom.append(new MechanismLigament2d("DRAWER SIDE", DRAWER_SIDE_LENGTH_METERS, 45, 6, new Color8Bit(Color.kGreen)));
    }

    /**
     * 
     */
    public void extend() {
        this.drawerArm.setLength(DRAWER_ARM_EXTENDED_LENGTH_METERS);
    }
    
    /**
     * 
     */
    public void retract() {
        this.drawerArm.setLength(DRAWER_ARM_RETRACTED_LENGTH_METERS);
    }
}
