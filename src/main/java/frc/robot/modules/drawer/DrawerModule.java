package frc.robot.modules.drawer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.subsystems.PneumaticSubsystem;

/**
 * 
 */
public class DrawerModule {
    private final int PING_ID = 5;
    private final int ECHO_ID = 4;

    private final DoubleSolenoid doubleSolenoid;
    private Ultrasonic ultrasonic;

    private double distance_millimeters;

    private final DrawerModuleMechanism drawerModuleMechanism;

    /**
     * 
     */
    public DrawerModule(PneumaticSubsystem pneumaticSubsystem) {
        this.doubleSolenoid = pneumaticSubsystem.getDoubleSolenoid(PneumaticConstants.DRAWER_OPEN_ID, PneumaticConstants.DRAWER_CLOSE_ID);
        this.ultrasonic = new Ultrasonic(PING_ID, ECHO_ID);
        this.ultrasonic.setEnabled(true);

        this.drawerModuleMechanism = new DrawerModuleMechanism();
    }

    /**
     * 
     */
    public void extend() {
        this.doubleSolenoid.set(Value.kForward);
        this.drawerModuleMechanism.extend();
    }

    /**
     * 
     */
    public void retract() {
        this.doubleSolenoid.set(Value.kReverse);
        this.drawerModuleMechanism.retract();
    }

    /**
     * 
     */
    public void update() {
        this.distance_millimeters = this.ultrasonic.getRangeMM();

        Logger.getInstance().recordOutput("Mechanisms/Drawer/Distance", this.distance_millimeters);
    }
}
