package frc.robot.modules.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;

public class ArmModule {
    /* Hardware Contants */
    private final int MOTOR_ID = 7;
    private final double DISTANCE_PER_PULSE = Math.PI * 2 / 120;
    private final double VOLTAGE_LIMIT = 8.0;

    private final int UPPER_LIMIT_SWITCH_ID = 9;
    // private final int LOWER_LIMIT_SWITCH_ID = 5;
    private final double CLIMB_SPEED = .65;
    private final double PID_TOLERANCE = Units.degreesToRadians(0.5);

    protected final PIDController pidController;

    /* Arm Hardware */
    private final TalonFX motor;
    private final DigitalInput upperLimitSwitch;
    // private final DigitalInput lowerLimitSwitch;
    
    /* Mechanisim2d Display for Monitoring the Arm Position */
    protected final ArmModuleMechanism armModuleMechanism = new ArmModuleMechanism();

    private double desired_angle_degrees;

    /**
     * 
     */
    public ArmModule() {
        this.motor = new TalonFX(MOTOR_ID);
        this.motor.setPosition(0.0);

        this.upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_ID);

        this.pidController = new PIDController(PIDConstants.ARM_MODULE_KP, PIDConstants.ARM_MODULE_KI, PIDConstants.ARM_MODULE_KD);
        this.pidController.setTolerance(PID_TOLERANCE);
    }

    /**
     * 
     */
    public void climb() {
        this.pidController.setSetpoint(0);
        this.motor.set(CLIMB_SPEED);
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
    public boolean isUpperLimitSwitchTriggered() {
        return this.upperLimitSwitch.get();
    }

    /**
     * 
     */
    public void stop() {
        this.motor.stopMotor();
    }

    /**
     * 
     */
    public void update() {
        if (isUpperLimitSwitchTriggered()) {
            setDesiredAngle(getCurrentAngle());
        }

        double pidOutput = this.pidController.calculate(getCurrentRadians(), Units.degreesToRadians(getDesiredAngle()));
        double voltage = MathUtil.clamp(pidOutput * VOLTAGE_LIMIT, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
        this.motor.setVoltage(voltage);

        Logger.recordOutput("Mechanisms/Arm/DesiredAngle", getDesiredAngle());
        Logger.recordOutput("Mechanisms/Arm/CurrentAngle", getCurrentAngle());
        Logger.recordOutput("Mechanisms/Arm/MotorPosition", getCurrentRadians());
        Logger.recordOutput("Mechanisms/Arm/PIDOutput", pidOutput);
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
        return Units.radiansToDegrees(getCurrentRadians());
    }

    public double getCurrentRadians() {
        return this.motor.getPosition().getValueAsDouble() * DISTANCE_PER_PULSE;
    }

    public double getDesiredAngle() {
        return this.desired_angle_degrees;
    }

    public void setDesiredAngle(double desired_angle_degrees) {
        this.desired_angle_degrees = desired_angle_degrees;
    }
}
