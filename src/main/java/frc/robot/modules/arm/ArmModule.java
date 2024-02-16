package frc.robot.modules.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.RobotConstants;
import frc.robot.utils.LoggedTunableNumber;

public class ArmModule {
    /* Hardware Contants */
    private final int MOTOR_ID = 7;
    private final int ENCODER_CHANNEL_A_ID = 0;
    private final int ENCODER_CHANNEL_B_ID = 1;
    private final double DISTANCE_PER_PULSE = 183 / 64;
    private final double MIN_RATE = 0.1;

    private final int UPPER_LIMIT_SWITCH_ID = 4;
    private final int LOWER_LIMIT_SWITCH_ID = 5;
    
    /* Motor PID Values */
    protected final double KP = 0.95 / 1000;
    protected final double KI = 0.0;
    protected final double KD = 0.0;

    /* Tunable PID */
    protected final LoggedTunableNumber kp = new LoggedTunableNumber("Arm/Kp", KP);
    protected final LoggedTunableNumber ki = new LoggedTunableNumber("Arm/Ki", KI);
    protected final LoggedTunableNumber kd = new LoggedTunableNumber("Arm/Kd", KD);

    protected final PIDController pidController = new PIDController(KP, KI, KD);

    /* Arm Hardware */
    private final TalonSRX motor;
    private final Encoder encoder;
    private final DigitalInput upperLimitSwitch;
    private final DigitalInput lowerLimitSwitch;
    
    /* Mechanisim2d Display for Monitoring the Arm Position */
    protected final ArmModuleMechanism armModuleMechanism = new ArmModuleMechanism();

    private double desired_angle_degrees;

    /**
     * 
     */
    public ArmModule() {
        this.motor = new TalonSRX(MOTOR_ID);

        this.encoder = new Encoder(ENCODER_CHANNEL_A_ID, ENCODER_CHANNEL_B_ID, false, Encoder.EncodingType.k4X);
        this.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.encoder.setMinRate(MIN_RATE);
        
        this.upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
        this.lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);
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
    public boolean isLowerLimitSwitchTriggered() {
        return !this.lowerLimitSwitch.get();
    }

    /**
     * 
     */
    public boolean isUpperLimitSwitchTriggered() {
        return !this.upperLimitSwitch.get();
    }

    /**
     * 
     */
    public void update() {
        if (RobotConstants.TUNING_MODE && (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kp.hasChanged(hashCode()))) {
            this.pidController.setPID(kp.get(), ki.get(), kd.get());
        }

        if (isUpperLimitSwitchTriggered()) {
            setDesiredAngle(getCurrentAngle());
        }
        
        if (isLowerLimitSwitchTriggered()) {
            this.encoder.reset();
            setDesiredAngle(getCurrentAngle());
        }

        double pidOutput = this.pidController.calculate(this.encoder.getDistance(), Units.degreesToRadians(getDesiredAngle()));
        this.motor.set(TalonSRXControlMode.PercentOutput, pidOutput);

        Logger.recordOutput("Mechanisms/Arm/Desired Angle", getDesiredAngle());
        Logger.recordOutput("Mechanisms/Arm/Current Angle", getCurrentAngle());
        Logger.recordOutput("Mechanisms/Arm/PID Output", pidOutput);
    }
    
    /**
     * Getters and Setters
     */
    public double getCurrentAngle() {
        return Units.radiansToDegrees(this.encoder.getDistance());
    }

    public double getDesiredAngle() {
        return this.desired_angle_degrees;
    }

    public void setDesiredAngle(double desired_angle_degrees) {
        this.desired_angle_degrees = desired_angle_degrees;
    }
}
