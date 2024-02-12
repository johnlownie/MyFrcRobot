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
    private final int MOTOR_ID = 5;
    private final int ENCODER_CHANNEL_1_ID = 0;
    private final int ENCODER_CHANNEL_2_ID = 1;
    private final double DISTANCE_PER_PULSE = 183 / 64;
    private final double MIN_RATE = 0.1;

    private final int UP_LIMIT_SWITCH_ID = 0;
    private final int DOWN_LIMIT_SWITCH_ID = 1;
    
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
    private final TalonSRX motor;
    private final Encoder encoder;
    private final DigitalInput upLimitSwitch;
    private final DigitalInput downLimitSwitch;
    
    /* Mechanisim2d Display for Monitoring the Arm Position */
    protected final ArmModuleMechanism armModuleMechanism = new ArmModuleMechanism();

    private boolean enabled;
    private double desired_angle_degrees;

    /**
     * 
     */
    public ArmModule() {
        this.motor = new TalonSRX(MOTOR_ID);

        this.encoder = new Encoder(ENCODER_CHANNEL_1_ID, ENCODER_CHANNEL_2_ID, false, Encoder.EncodingType.k4X);
        this.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        this.encoder.setMinRate(MIN_RATE);
        
        this.upLimitSwitch = new DigitalInput(UP_LIMIT_SWITCH_ID);
        this.downLimitSwitch = new DigitalInput(DOWN_LIMIT_SWITCH_ID);

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
    public boolean isDownLimitSwitchTriggered() {
        return !this.downLimitSwitch.get();
    }

    /**
     * 
     */
    public boolean isUpLimitSwitchTriggered() {
        return !this.upLimitSwitch.get();
    }

    /**
     * 
     */
    public void update() {
        if (RobotConstants.TUNING_MODE && (kp.hasChanged(hashCode()) || ki.hasChanged(hashCode()) || kp.hasChanged(hashCode()))) {
            this.pidController.setPID(kp.get(), ki.get(), kd.get());
        }

        if (isUpLimitSwitchTriggered()) {
            setDesiredAngle(getCurrentAngle());
        }
        
        if (isDownLimitSwitchTriggered()) {
            this.encoder.reset();
            setDesiredAngle(getCurrentAngle());
        }

        double pidOutput = isEnabled() ? this.pidController.calculate(this.encoder.getDistance(), Units.degreesToRadians(getDesiredAngle())) : 0.0;
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
