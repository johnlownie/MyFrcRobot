package frc.robot.modules.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants;
import frc.robot.utils.LoggedTunableNumber;

public class ArmModule {
    /* Hardware Contants */
    private final int MOTOR_ID = 5;
    
    private final int INTAKE_LIMIT_SWITCH_ID = 0;
    private final int BACK_LIMIT_SWITCH_ID = 1;
    
    /* Motor PID Values */
    private final double KP = 0.95 / 1000;
    private final double KI = 0.0;
    private final double KD = 0.0;

    /* Tunable PID */
    private final LoggedTunableNumber kp = new LoggedTunableNumber("Arm/Kp", KP);
    private final LoggedTunableNumber ki = new LoggedTunableNumber("Arm/Ki", KI);
    private final LoggedTunableNumber kd = new LoggedTunableNumber("Arm/Kd", KD);

    private final PIDController pidController = new PIDController(KP, KI, KD);

    /* Arm Hardware */
    private final TalonSRX armMotor;
    private final DigitalInput intakeLimitSwitch;
    private final DigitalInput backLimitSwitch;

    /* Mechanisim2d Display for Monitoring the Arm Position */
    protected final ArmModuleMechanism armModuleMechanism = new ArmModuleMechanism();

    private boolean enabled;
    private double desired_angle_degrees;

    /**
     * 
     */
    public ArmModule() {
        this.armMotor = new TalonSRX(MOTOR_ID);
        
        this.intakeLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);
        this.backLimitSwitch = new DigitalInput(BACK_LIMIT_SWITCH_ID);

        this.enabled = false;
    }

    /**
     * 
     */
    public boolean isAtAngle() {
        return isAtAngle(getDesiredAngle());
    }

    /**
     * 
     */
    public boolean isAtAngle(double angle) {
        return false;
    }

    /**
     * 
     */
    public boolean isBackLimitSwitchTriggered() {
        return !this.backLimitSwitch.get();
    }

    /**
     * 
     */
    public boolean isIntakeLimitSwitchTriggered() {
        return !this.intakeLimitSwitch.get();
    }

    /**
     * 
     */
    public void update() {
        if (RobotConstants.TUNING_MODE && (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kp.hasChanged(hashCode()))) {
            this.pidController.setPID(kp.get(), ki.get(), kd.get());
        }
    }
    
    /**
     * Getters and Setters
     */
    public double getDesiredAngle() {
        return this.desired_angle_degrees;
    }

    public boolean isEnabled() {
        return this.enabled;
    }

    public void setDesiredAngle(double desired_angle_degrees) {
        this.desired_angle_degrees = desired_angle_degrees;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
