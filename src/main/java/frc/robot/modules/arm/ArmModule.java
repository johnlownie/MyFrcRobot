package frc.robot.modules.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;

public class ArmModule {
    /* Hardware Contants */
    private final int MOTOR_ID = 7;
    private final int ENCODER_CHANNEL_A_ID = 0;
    private final int ENCODER_CHANNEL_B_ID = 1;
    private final double DISTANCE_PER_PULSE = 183 / 64;
    private final double MIN_RATE = 0.1;

    private final int UPPER_LIMIT_SWITCH_ID = 4;
    private final int LOWER_LIMIT_SWITCH_ID = 5;

    protected final PIDController pidController;

    /* Arm Hardware */
    private final TalonFX motor;
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
        this.motor = new TalonFX(MOTOR_ID);
        this.motor.setPosition(0.0);

        this.encoder = new Encoder(ENCODER_CHANNEL_A_ID, ENCODER_CHANNEL_B_ID, false, Encoder.EncodingType.k4X);
        this.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.encoder.setMinRate(MIN_RATE);
        
        this.upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);
        this.lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_ID);

        this.pidController = new PIDController(PIDConstants.ARM_MODULE_KP, PIDConstants.ARM_MODULE_KI, PIDConstants.ARM_MODULE_KD);
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
        double difference = getCurrentAngle() - angle;

        return -1.0 <= difference && difference <= 1.0;
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
        if (isUpperLimitSwitchTriggered()) {
            setDesiredAngle(getCurrentAngle());
        }
        
        if (isLowerLimitSwitchTriggered()) {
            this.encoder.reset();
            setDesiredAngle(getCurrentAngle());
        }

        double pidOutput = this.pidController.calculate(this.encoder.getDistance(), Units.degreesToRadians(getDesiredAngle()));

        double voltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
        this.motor.setVoltage(voltage);

        Logger.recordOutput("Mechanisms/Arm/Desired Angle", getDesiredAngle());
        Logger.recordOutput("Mechanisms/Arm/Current Angle", getCurrentAngle());
        Logger.recordOutput("Mechanisms/Arm/PID Output", pidOutput);
    }

    /**
     * 
     */
    public void updatePID(double kP, double kI, double kD) {
        if (RobotConstants.TUNING_MODE) {
            this.pidController.setPID(kP, kI, kD);
        }
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
