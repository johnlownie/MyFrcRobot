package frc.robot.modules.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.PIDConstants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterModule {
    /* Hardware Contants */
    private final int LEFT_MOTOR_ID = 62;
    private final int RIGHT_MOTOR_ID = 61;
    private final int KICKER_MOTOR_ID = 60;
    private final int INTAKE_LIMIT_SWITCH_ID = 8;

    private final double GEAR_RATIO = 1.0;
    private final int CURRENT_LIMIT_AMPS = 20;
    private final double MAX_OUTPUT = 1.0;

    private final double KICKER_INTAKE_SPEED = 0.6;
    private final double KICKER_PULLBACK_SPEED = -0.1;

    private final double REVERSE_SPEED = -0.05;
    private final int AMP_VELOCITY = 2000;
    private final int SPEAKER_VELOCITY = 5400;

    /* Shooter Hardware */
    protected final CANSparkMax leftMotor;
    protected final CANSparkMax rightMotor;
    protected final CANSparkMax kickerMotor;
    private final DigitalInput intakeLimitSwitch;

    protected final RelativeEncoder leftEncoder;
    protected final RelativeEncoder rightEncoder;

    private final SparkPIDController leftPidController;
    private final SparkPIDController rightPidController;

    /**
     * 
     */
    public ShooterModule() {
        this.leftMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        this.leftMotor.restoreFactoryDefaults();
        this.leftMotor.setInverted(true);
        this.leftMotor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
        
        this.rightMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
        this.rightMotor.restoreFactoryDefaults();
        this.rightMotor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);

        this.kickerMotor = new CANSparkMax(KICKER_MOTOR_ID, MotorType.kBrushless);
        this.kickerMotor.setInverted(true);

        this.intakeLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);

        this.leftEncoder = this.leftMotor.getEncoder();
        this.leftEncoder.setVelocityConversionFactor(1.0);
        this.leftEncoder.setPositionConversionFactor(1.0 / GEAR_RATIO);

        this.rightEncoder = this.rightMotor.getEncoder();
        this.rightEncoder.setVelocityConversionFactor(1.0);
        this.rightEncoder.setPositionConversionFactor(1.0 / GEAR_RATIO);

        this.leftPidController = this.leftMotor.getPIDController();
        this.leftPidController.setP(PIDConstants.SHOOTER_MODULE_KP);
        this.leftPidController.setD(PIDConstants.SHOOTER_MODULE_KD);
        this.leftPidController.setFF(PIDConstants.SHOOTER_MODULE_KF);
        this.leftPidController.setOutputRange(-MAX_OUTPUT, MAX_OUTPUT);

        this.rightPidController = this.rightMotor.getPIDController();
        this.rightPidController.setP(PIDConstants.SHOOTER_MODULE_KP);
        this.rightPidController.setD(PIDConstants.SHOOTER_MODULE_KD);
        this.rightPidController.setFF(PIDConstants.SHOOTER_MODULE_KF);
        this.rightPidController.setOutputRange(-MAX_OUTPUT, MAX_OUTPUT);
    }

    /**
     * 
     */
    public boolean hasNote() {
        return this.intakeLimitSwitch.get();
    }

    /**
     * 
     */
    public void intake() {
        this.leftMotor.set(REVERSE_SPEED);
        this.rightMotor.set(REVERSE_SPEED);
        this.kickerMotor.set(KICKER_INTAKE_SPEED);
    }

    /**
     * 
     */
    public boolean isSpunupForAmp() {
        return this.leftEncoder.getVelocity() >= AMP_VELOCITY && this.rightEncoder.getVelocity() >= AMP_VELOCITY;
    }

    /**
     * 
     */
    public boolean isSpunupForSpeaker() {
        return this.leftEncoder.getVelocity() >= SPEAKER_VELOCITY && this.rightEncoder.getVelocity() >= SPEAKER_VELOCITY;
    }

    /**
     * 
     */
    public void pullback() {
        this.kickerMotor.set(KICKER_PULLBACK_SPEED);
    }

    /**
     * 
     */
    public void shoot() {
        this.kickerMotor.set(KICKER_INTAKE_SPEED);
    }

    /**
     * 
     */
    public void spinupForAmp() {
        this.leftPidController.setReference(AMP_VELOCITY, ControlType.kVelocity);
        this.rightPidController.setReference(AMP_VELOCITY, ControlType.kVelocity);
    }

    /**
     * 
     */
    public void spinupForSpeaker() {
        this.leftPidController.setReference(SPEAKER_VELOCITY, ControlType.kVelocity);
        this.rightPidController.setReference(SPEAKER_VELOCITY, ControlType.kVelocity);
    }

    /**
     * 
     */
    public void stop() {
        this.leftPidController.setReference(0.0, ControlType.kVelocity);
        this.rightPidController.setReference(0.0, ControlType.kVelocity);
        this.kickerMotor.set(0.0);
    }

    /**
     * 
     */
    public void stopKicker() {
        this.kickerMotor.set(0.0);
    }

    /**
     * 
     */
    public void update() {
        Logger.recordOutput("Mechanisms/Shooter/Left Velocity", this.leftEncoder.getVelocity());
        Logger.recordOutput("Mechanisms/Shooter/Right Velocity", this.rightEncoder.getVelocity());
        Logger.recordOutput("Mechanisms/Shooter/Kicker Output Current", this.kickerMotor.getOutputCurrent());
    }

    /**
     * Getters and Setters - used only in simulation
     */
    public void setHasNote(boolean has_note) {}
}
