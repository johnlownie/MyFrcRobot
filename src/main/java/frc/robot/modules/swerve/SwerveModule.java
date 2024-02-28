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
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.Conversions;

/**
 * 
 */
public class SwerveModule {
    /* Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.32; // / 12.0;
    private final double DRIVE_KV = 1.51; // / 12.0;
    private final double DRIVE_KA = 0.27; // / 12.0;

    // Not sure what these represent, but smaller is faster
    private final double MOTION_MAGIC_VELOCITY = .125;
    private final double MOTION_MAGIC_ACCELERATION = .0625;

    // Default S
    public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
    
    /* Swerve Hardware */
    protected final TalonFX angleMotor;
    protected final TalonFX driveMotor;
    protected final CANcoder encoder;
    
    protected final Rotation2d angleOffset;

    /* Module Variables */
    protected final int module_id;
    
    private final SimpleMotorFeedforward feedForward;
    private final PositionVoltage positionVoltage;
    private final VelocityVoltage velocityVoltage;

    /**
     * 
     */
    public SwerveModule(int module_id, int drive_motor_id, int angle_motor_id, int can_coder_id, Rotation2d angleOffset) {
        this.module_id = module_id;
        this.driveMotor = getDriveMotor(drive_motor_id);
        this.angleMotor = getAngleMotor(angle_motor_id);
        this.encoder = getEncoder(can_coder_id);
        this.angleOffset = angleOffset;
        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
        this.positionVoltage = new PositionVoltage(0);
        this.velocityVoltage = new VelocityVoltage(0);
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

        talonFXConfiguration.Slot0.kP = PIDConstants.SWERVE_MODULE_TURN_KP;
        talonFXConfiguration.Slot0.kI = PIDConstants.SWERVE_MODULE_TURN_KI;
        talonFXConfiguration.Slot0.kD = PIDConstants.SWERVE_MODULE_TURN_KD;

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

        talonFXConfiguration.Slot0.kP = PIDConstants.SWERVE_MODULE_DRIVE_KP;
        talonFXConfiguration.Slot0.kI = PIDConstants.SWERVE_MODULE_DRIVE_KI;
        talonFXConfiguration.Slot0.kD = PIDConstants.SWERVE_MODULE_DRIVE_KD;

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
            Conversions.RPSToMPS(this.driveMotor.getVelocity().getValue(), CHOSEN_MODULE.wheelCircumference), 
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
        PositionVoltage positionVoltage = this.positionVoltage.withPosition(desiredState.angle.getRotations());
        this.angleMotor.setControl(positionVoltage);

        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/TurnVVPosition", positionVoltage.Position);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/TurnVVVelocity", positionVoltage.Velocity);
        Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/TurnVVFeedforward", positionVoltage.FeedForward);
    }

    /**
     * 
     */
    protected void setDriveState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            double output = desiredState.speedMetersPerSecond / SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
            this.driveMotor.setControl(new DutyCycleOut(output));

            Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/DriveOutput", output);
        }
        else {
            double radiansPerSecond = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, CHOSEN_MODULE.wheelCircumference);
            VelocityVoltage velocityVoltage = this.velocityVoltage.withVelocity(radiansPerSecond);
            velocityVoltage.FeedForward = this.feedForward.calculate(desiredState.speedMetersPerSecond);
            this.driveMotor.setControl(velocityVoltage);

            Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/DriveVVAcceleration", velocityVoltage.Acceleration);
            Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/DriveVVVelocity", velocityVoltage.Velocity);
            Logger.recordOutput("Mechanisms/SwerveModules/Mod" + this.module_id + "/DriveVVFeedForward", velocityVoltage.FeedForward);
        }
    }

    /**
     * Only used in simulation
     */
    public void updatePositions() {
    }

    /**
     * 
     */
    public void updateDrivePID(double kP, double kI, double kD) {
        if (RobotConstants.TUNING_MODE) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = kP;
            slot0Configs.kI = kI;
            slot0Configs.kD = kD;
            this.driveMotor.getConfigurator().refresh(slot0Configs);
        }
    }

    /**
     * 
     */
    public void updateTurnPID(double kP, double kI, double kD) {
        if (RobotConstants.TUNING_MODE) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = kP;
            slot0Configs.kI = kI;
            slot0Configs.kD = kD;
            this.angleMotor.getConfigurator().refresh(slot0Configs);
        }
    }
}
