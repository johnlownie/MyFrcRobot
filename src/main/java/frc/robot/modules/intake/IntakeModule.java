package frc.robot.modules.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * 
 */
public class IntakeModule {
    /* Hardware Contants */
    private final int MOTOR_CHANNEL = 10;
    private final double MOTOR_INTAKE_SPEED = 0.8;
    private final double MOTOR_OUTTAKE_SPEED = -0.6;

    /* Intake Hardware */
    protected final TalonSRX motor;

    /**
     * 
     */
    public IntakeModule() {
        this.motor = new TalonSRX(MOTOR_CHANNEL);
        this.motor.setInverted(true);
    }

    /**
     * 
     */
    public void eject() {
        this.motor.set(TalonSRXControlMode.PercentOutput, MOTOR_OUTTAKE_SPEED);
    }

    /**
     * 
     */
    public void intake() {
        this.motor.set(TalonSRXControlMode.PercentOutput, MOTOR_INTAKE_SPEED);
    }

    /**
     * 
     */
    public void stop() {
        this.motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    /**
     * 
     */
    public void update() {
        Logger.recordOutput("Mechanisms/Intake/OutputPercent", this.motor.getMotorOutputPercent());
        Logger.recordOutput("Mechanisms/Intake/OutputVoltage", this.motor.getMotorOutputVoltage());
    }
}
