package frc.robot.modules.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Conversions;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class SwerveModule {
    /* Drive Motor PID Values */
    private final double DRIVE_KP = 0.05;
    private final double DRIVE_KI = 0.0;
    private final double DRIVE_KD = 0.0;

    /* Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.32 / 12.0;
    private final double DRIVE_KV = 1.51 / 12.0;
    private final double DRIVE_KA = 0.27 / 12.0;

    // Not sure what these represent, but smaller is faster
    private final double MOTION_MAGIC_VELOCITY = .125;
    private final double MOTION_MAGIC_ACCELERATION = .0625;

    // Default S
    public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drive motor control requests */
    protected final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* Angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

    /* Tunable PID */
    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp", DRIVE_KP);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drive/DriveKi", DRIVE_KI);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd", DRIVE_KD);

    private final LoggedTunableNumber angleKp = new LoggedTunableNumber("Drive/TurnKp", CHOSEN_MODULE.angleKP);
    private final LoggedTunableNumber angleKi = new LoggedTunableNumber("Drive/TurnKi", CHOSEN_MODULE.angleKI);
    private final LoggedTunableNumber angleKd = new LoggedTunableNumber("Drive/TurnKd", CHOSEN_MODULE.angleKD);

    /* Motors */
    protected final int module_id;
    protected final TalonFX angleMotor;
    protected final TalonFX driveMotor;
 
    protected final CANcoder encoder;

    protected final Rotation2d angleOffset;

    /* Variables */
    protected double angleAbsolutePositionDeg;
    protected double anglePositionDeg;
    protected double angleVelocityRevPerMin;
    protected double angleLastAngle;

    /**
     * 
     */
    public SwerveModule(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angleOffset) {
        this.module_id = module_id;
        this.driveMotor = getDriveMotor(drive_motor_id);
        this.angleMotor = getAngleMotor(angle_motor_id);
        this.encoder = getEncoder(can_coder_id);
        this.angleOffset = angleOffset;
    }

    /**
     * 
     */
    private TalonFX getAngleMotor(int motor_id) {
        TalonFX motor = new TalonFX(motor_id);
        TalonFXConfiguration talonFXConfiguration = getAngleMotorConfiguration();

        motor.getConfigurator().apply(talonFXConfiguration);

        return motor;
    }

    /**
     * 
     */
    private TalonFXConfiguration getAngleMotorConfiguration() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = CHOSEN_MODULE.angleMotorInvert;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfiguration.Feedback.SensorToMechanismRatio = CHOSEN_MODULE.angleGearRatio;
        talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.ANGLE_CURRENT_LIMIT;
        talonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants.ANGLE_CURRENT_THRESHOLD;
        talonFXConfiguration.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants.ANGLE_TIME_THRESHOLD;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.STATOR_CURRENT_LIMIT;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfiguration.Slot0.kP = angleKp.get();
        talonFXConfiguration.Slot0.kI = angleKi.get();
        talonFXConfiguration.Slot0.kD = angleKd.get();

        // talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2.0 / MOTION_MAGIC_VELOCITY / this.angleEncoderVelocityCoefficient;
        // talonFXConfiguration.MotionMagic.MotionMagicAcceleration = (8.0 - 2.0) / MOTION_MAGIC_ACCELERATION / this.angleEncoderVelocityCoefficient;
    
        return talonFXConfiguration;
    }
    
    /**
     * 
     */
    private TalonFX getDriveMotor(int motor_id) {
        TalonFX motor = new TalonFX(motor_id);
        TalonFXConfiguration talonFXConfiguration = getDriveMotorConfiguration();

        motor.getConfigurator().apply(talonFXConfiguration);

        return motor;
    }

    /**
     * 
     */
    private TalonFXConfiguration getDriveMotorConfiguration() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.MotorOutput.Inverted = CHOSEN_MODULE.driveMotorInvert;
        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        talonFXConfiguration.Feedback.SensorToMechanismRatio = CHOSEN_MODULE.driveGearRatio;

        talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.DRIVE_CURRENT_LIMIT;
        talonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants.DRIVE_CURRENT_THRESHOLD;
        talonFXConfiguration.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants.DRIVE_TIME_THRESHOLD;
        talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.STATOR_CURRENT_LIMIT;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXConfiguration.Slot0.kP = driveKp.get();
        talonFXConfiguration.Slot0.kI = driveKi.get();
        talonFXConfiguration.Slot0.kD = driveKd.get();

        talonFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveModuleConstants.OPEN_LOOP_RAMP;
        talonFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveModuleConstants.OPEN_LOOP_RAMP;

        talonFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveModuleConstants.CLOSED_LOOP_RAMP;
        talonFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveModuleConstants.CLOSED_LOOP_RAMP;

        return talonFXConfiguration;
    }

    /**
     * 
     */
    private CANcoder getEncoder(int can_coder_id) {
        CANcoder canCoder = new CANcoder(can_coder_id);
        CANcoderConfiguration canCoderConfiguration = getEncoderConfiguration();

        canCoder.getConfigurator().apply(canCoderConfiguration);

        canCoder.getPosition().setUpdateFrequency(SwerveModuleConstants.CANCODER_UPDATE_FREQUENCY);
        canCoder.getVelocity().setUpdateFrequency(SwerveModuleConstants.CANCODER_UPDATE_FREQUENCY);

        return canCoder;
    }

    /**
     * 
     */
    private CANcoderConfiguration getEncoderConfiguration() {
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.SensorDirection = CHOSEN_MODULE.cancoderInvert;

        return canCoderConfiguration;
    }

    /**
     * 
     */
    public int getModuleId() {
        return this.module_id;
    }

    /**
     * 
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.RPSToMPS(this.driveMotor.getPosition().getValue(), CHOSEN_MODULE.wheelCircumference), 
                Rotation2d.fromRotations(this.angleMotor.getPosition().getValue())
        );
    }

    /**
     * 
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.velocityMPS(this.driveMotor.getVelocity().getValue(), CHOSEN_MODULE.wheelCircumference), 
            Rotation2d.fromRotations(this.angleMotor.getPosition().getValue())
        );
    }

    /**
     * 
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        setAngleState(desiredState);
        setDriveState(desiredState, isOpenLoop);
    }

    /**
     * 
     */
    protected void setAngleState(SwerveModuleState desiredState) {
        setAngleVoltage(this.anglePosition.withPosition(desiredState.angle.getRotations()));
    }

    /**
     * 
     */
    protected void setAngleVoltage(PositionVoltage positionVoltage) {
        this.angleMotor.setControl(positionVoltage);
    }

    /**
     * 
     */
    protected void setDriveState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            this.driveDutyCycle.Output = desiredState.speedMetersPerSecond / SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
            setDriveVoltage(driveDutyCycle);
        }
        else {
            this.driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, CHOSEN_MODULE.wheelCircumference);
            this.driveVelocity.FeedForward = this.feedForward.calculate(desiredState.speedMetersPerSecond);
            setDriveVelocity(driveVelocity);
        }
    }

    /**
     * 
     */
    protected void setDriveVelocity(VelocityVoltage velocityVoltage) {
        this.driveMotor.setControl(velocityVoltage);
    }

    /**
     * 
     */
    protected void setDriveVoltage(DutyCycleOut dutyCycleOut) {
        this.driveMotor.setControl(dutyCycleOut);
    }

    public void updatePositions() {
        if (RobotConstants.TUNING_MODE) {
            if (driveKp.hasChanged(hashCode()) || driveKi.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
                Slot0Configs slot0Configs = new Slot0Configs();
                slot0Configs.kP = this.driveKp.get();
                slot0Configs.kI = this.driveKi.get();
                slot0Configs.kD = this.driveKd.get();
                this.driveMotor.getConfigurator().refresh(slot0Configs);
            }
            
            if (angleKp.hasChanged(hashCode()) || angleKi.hasChanged(hashCode()) || angleKd.hasChanged(hashCode())) {
                Slot0Configs slot0Configs = new Slot0Configs();
                slot0Configs.kP = this.angleKp.get();
                slot0Configs.kI = this.angleKi.get();
                slot0Configs.kD = this.angleKd.get();
                this.angleMotor.getConfigurator().refresh(slot0Configs);
            }
        }
    }
}
