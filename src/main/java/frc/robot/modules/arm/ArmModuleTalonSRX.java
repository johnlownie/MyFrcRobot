package frc.robot.modules.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class ArmModuleTalonSRX extends ArmModule {
    /* Hardware Contants */
    private final int LOWER_MOTOR_ID = 9;
    private final int UPPER_MOTOR_ID = 1;
    
    private final int INNER_LIMIT_SWITCH_ID = 9;
    private final int OUTER_LIMIT_SWITCH_ID = 8;
    
    /* Motor PID Values */
    private final double KP = 0.95 / 1000;
    private final double KI = 0.0;
    private final double KD = 0.0;

    /* Tunable PID */
    private final LoggedTunableNumber kp = new LoggedTunableNumber("Arm/Kp", KP);
    private final LoggedTunableNumber ki = new LoggedTunableNumber("Arm/Ki", KI);
    private final LoggedTunableNumber kd = new LoggedTunableNumber("Arm/Kd", KD);

    private final PIDController pidController = new PIDController(KP, KI, KD);

    /* Gripper Hardware */
    private DoubleSolenoid doubleSolenoid;

    /* Arm Hardware */
    private final WPI_TalonSRX lowerMotor;
    private final WPI_TalonSRX upperMotor;
    private final DigitalInput innerLimitSwitch;
    private final DigitalInput outerLimitSwitch;
    
    /**
     * 
     */
    public ArmModuleTalonSRX(PneumaticSubsystem pneumaticSubsystem) {
        super();

        this.lowerMotor = new WPI_TalonSRX(LOWER_MOTOR_ID);
        this.upperMotor = new WPI_TalonSRX(UPPER_MOTOR_ID);
        this.lowerMotor.follow(this.upperMotor);
        
        this.innerLimitSwitch = new DigitalInput(INNER_LIMIT_SWITCH_ID);
        this.outerLimitSwitch = new DigitalInput(OUTER_LIMIT_SWITCH_ID);

        this.doubleSolenoid = pneumaticSubsystem.getDoubleSolenoid(PneumaticConstants.GRIPPER_OPEN_ID, PneumaticConstants.GRIPPER_CLOSE_ID);
    }

    @Override
    public boolean atAngle() {
        return true;
    }

    @Override
    public boolean atAngle(double angle) {
        return true;
    }

    @Override
    public void close() {

    }

    @Override
    public boolean isInnerLimitSwitchTriggered() {
        return !this.innerLimitSwitch.get();
    }

    @Override
    public boolean isOuterLimitSwitchTriggered() {
        return !this.outerLimitSwitch.get();
    }

    @Override
    public void open() {

    }

    @Override
    public void resetController() {

    }

    /**
     * 
     */
    public void set(Value value) {
        this.doubleSolenoid.set(value);
    }

    @Override
    public void setDesiredAngle(double angle_in_degrees) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void update() {
        if (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kp.hasChanged(hashCode())) {
            this.pidController.setP(kp.get());
            this.pidController.setI(ki.get());
            this.pidController.setD(kd.get());
        }


    }
}
