package frc.robot.modules.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants.MK4I_L2;
import frc.robot.utils.Conversions;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class SwerveModuleTalonFX extends SwerveModule {
    /* PID Slot Index */
    private final int SLOT_INDEX = 0;

    /* Drive Motor PID Values */
    private final double DRIVE_KP = 0.8;
    private final double DRIVE_KI = 0.0;
    private final double DRIVE_KD = 0.0;

    /* Turn Motor PID Values */
    private final double TURN_KP = 12.0;
    private final double TURN_KI = 0.0;
    private final double TURN_KD = 0.0;

    /* Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.116970;
    private final double DRIVE_KV = 0.133240;
    private final double DRIVE_KA = 0.0;

    // Not sure what these represent, but smaller is faster
    private final double MOTION_MAGIC_VELOCITY = .125;
    private final double MOTION_MAGIC_ACCELERATION = .0625;

    /* Tunable PID */
    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp", DRIVE_KP);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drive/DriveKi", DRIVE_KI);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd", DRIVE_KD);

    private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/TurnKp", TURN_KP);
    private final LoggedTunableNumber turnKi = new LoggedTunableNumber("Drive/TurnKi", TURN_KI);
    private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd", TURN_KD);

    /* Motors */
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turnMotor;
 
    private final double drivePositionCoefficient;
    private final double driveVelocityCoefficient;

    private final double turnEncoderPositionCoefficient;
    private final double turnEncoderVelocityCoefficient;
    private final CANCoder encoder;

    private final SimpleMotorFeedforward feedForward;

    /**
     * 
     */
    public SwerveModuleTalonFX(int module_id, int drive_motor_id, int turn_motor_id, int can_coder_id, double angle_offset_degrees) {
        super(module_id);
        
        this.driveMotor = getDriveMotor(drive_motor_id);
        this.drivePositionCoefficient = Math.PI * MK4I_L2.WHEEL_DIAMETER_METERS * MK4I_L2.DRIVE_GEAR_RATIO / MK4I_L2.TICKS_PER_ROTATION;
        this.driveVelocityCoefficient = this.drivePositionCoefficient * 10.0;
        
        this.turnMotor = getTurnMotor(turn_motor_id);
        this.turnEncoderPositionCoefficient = 2.0 * Math.PI / MK4I_L2.TICKS_PER_ROTATION * MK4I_L2.ANGLE_GEAR_RATIO;
        this.turnEncoderVelocityCoefficient = this.turnEncoderPositionCoefficient * 10.0;
        
        this.encoder = getEncoder(can_coder_id, angle_offset_degrees);

        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
    }

    @Override
    protected void applyDriveSettings() {

    }

    @Override
    protected void applyTurnSettings() {

    }

    /**
     * Configures the motor offset from the CANCoder's abosolute position. In an ideal state, this only needs to happen
     * once. However, sometime it fails and we end up with a wheel that isn't in the right position.
     * See https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99
     * @return the absolute angle
     */
    public double configMotorOffset(boolean logErrors) {
        double angle = getAbsoluteAngle();
        var angleErrorCode = this.encoder.getLastError();

        if (angleErrorCode == ErrorCode.OK) {
            this.driveMotor.setSelectedSensorPosition(angle / this.drivePositionCoefficient, SLOT_INDEX, MK4I_L2.CAN_TIMEOUT_MS);
        }

        return angle;
    }

    /**
     * 
     */
    private double getAbsoluteAngle() {
        double angle = this.encoder.getPosition();
    
        angle = Math.toRadians(angle);
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
          angle += 2.0 * Math.PI;
        }
    
        return angle;
    }
    
    /**
     * 
     */
    private WPI_TalonFX getDriveMotor(int motor_id) {
        WPI_TalonFX motor = new WPI_TalonFX(motor_id);
        TalonFXConfiguration talonFXConfiguration = getDriveMotorConfiguration();

        CtreUtils.checkCtreError(motor.configAllSettings(talonFXConfiguration), "Failed to configure Falcon 500");

        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(NeutralMode.Coast);
        // motor.setInverted(moduleConfiguration.isDriveInverted());
        motor.setSensorPhase(true);
        motor.setSafetyEnabled(true);

        return motor;
    }

    /**
     * 
     */
    private TalonFXConfiguration getDriveMotorConfiguration() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.voltageCompSaturation = 12;
        talonFXConfiguration.supplyCurrLimit.currentLimit = 80;
        talonFXConfiguration.supplyCurrLimit.enable = true;
    
        talonFXConfiguration.slot0.kP = driveKp.get();
        talonFXConfiguration.slot0.kI = driveKi.get();
        talonFXConfiguration.slot0.kD = driveKd.get();

        return talonFXConfiguration;
    }

    /**
     * 
     */
    private CANCoder getEncoder(int can_coder_id, double angle_offset_degrees) {
        CANCoder canCoder = new CANCoder(can_coder_id);
        CANCoderConfiguration canCoderConfiguration = getEncoderConfiguration(angle_offset_degrees);

        CtreUtils.checkCtreError(canCoder.configAllSettings(canCoderConfiguration, MK4I_L2.CAN_TIMEOUT_MS), "Failed to configure CANCoder");
        CtreUtils.checkCtreError(canCoder.setPositionToAbsolute(MK4I_L2.CAN_TIMEOUT_MS), "Failed to set CANCoder to absolute");
        CtreUtils.checkCtreError(canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, MK4I_L2.CAN_TIMEOUT_MS), "Failed to configure CANCoder update rate");

        return canCoder;
    }

    /**
     * 
     */
    private CANCoderConfiguration getEncoderConfiguration(double angle_offset_degrees) {
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.magnetOffsetDegrees = Math.toDegrees(angle_offset_degrees);
        canCoderConfiguration.sensorDirection = false;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        return canCoderConfiguration;
    }

    /**
     * 
     */
    private WPI_TalonFX getTurnMotor(int motor_id) {
        WPI_TalonFX motor = new WPI_TalonFX(motor_id);
        TalonFXConfiguration talonFXConfiguration = getTurnMotorConfiguration();

        CtreUtils.checkCtreError(motor.configAllSettings(talonFXConfiguration, MK4I_L2.CAN_TIMEOUT_MS), "Failed to configure Falcon 500 settings");

        motor.enableVoltageCompensation(true);
        motor.setSensorPhase(true);
        motor.setInverted(MK4I_L2.ANGLE_MOTOR_INVERTED ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        motor.setNeutralMode(NeutralMode.Coast);
    
        return motor;
    }

    /**
     * 
     */
    private TalonFXConfiguration getTurnMotorConfiguration() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.voltageCompSaturation = 12;
        talonFXConfiguration.supplyCurrLimit.currentLimit = 80;
        talonFXConfiguration.supplyCurrLimit.enable = true;
    
        talonFXConfiguration.slot0.kP = turnKp.get();
        talonFXConfiguration.slot0.kI = turnKi.get();
        talonFXConfiguration.slot0.kD = turnKd.get();

        talonFXConfiguration.slot0.kF = (1023.0 * this.turnEncoderPositionCoefficient / 12) * MOTION_MAGIC_VELOCITY;
        talonFXConfiguration.motionCruiseVelocity = 2.0 / MOTION_MAGIC_VELOCITY / this.turnEncoderVelocityCoefficient;
        talonFXConfiguration.motionAcceleration = (8.0 - 2.0) / MOTION_MAGIC_ACCELERATION / this.turnEncoderVelocityCoefficient;
    
        return talonFXConfiguration;
    }

    @Override
    public void reseedSteerMotorOffset() {
        configMotorOffset(false);
    }

    @Override
    public void resetPIDController() {
        this.driveMotor.config_kP(SLOT_INDEX, driveKp.get());
        this.driveMotor.config_kI(SLOT_INDEX, driveKi.get());
        this.driveMotor.config_kD(SLOT_INDEX, driveKd.get());

        this.turnMotor.config_kP(SLOT_INDEX, turnKp.get());
        this.turnMotor.config_kI(SLOT_INDEX, turnKi.get());
        this.turnMotor.config_kD(SLOT_INDEX, turnKd.get());
    }

    @Override
    public void setAnglePosition(double degrees) {
        this.turnMotor.set(ControlMode.Position, Conversions.degreesTo(degrees, MK4I_L2.ANGLE_GEAR_RATIO));
    }

    @Override
    protected void setDrivePercentage(double percentage) {
        this.driveAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
        this.driveMotor.set(ControlMode.PercentOutput, this.driveAppliedVolts);
    }

    @Override
    protected void setDriveVelocity(double velocity) {
        double wheelRPM = ((velocity * 60) / MK4I_L2.WHEEL_CIRCUMFERENCE);
        double motorRPM = wheelRPM * MK4I_L2.DRIVE_GEAR_RATIO;
        motorRPM = motorRPM * MK4I_L2.TICKS_PER_ROTATION / 600;

        this.driveAppliedPercentage = MathUtil.clamp(this.feedForward.calculate(velocity), 0, 1);

        this.driveMotor.set(ControlMode.Velocity, motorRPM, DemandType.ArbitraryFeedForward, this.driveAppliedPercentage);
    }

    @Override 
    public void updatePositions() {
        if (RobotConstants.TUNING_MODE) {
            if (driveKp.hasChanged(hashCode()) || driveKi.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
                this.driveMotor.config_kP(SLOT_INDEX, this.driveKp.get());
                this.driveMotor.config_kI(SLOT_INDEX, this.driveKi.get());
                this.driveMotor.config_kD(SLOT_INDEX, this.driveKd.get());
            }
            
            if (turnKp.hasChanged(hashCode()) || turnKi.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
                this.turnMotor.config_kP(SLOT_INDEX, this.turnKp.get());
                this.turnMotor.config_kI(SLOT_INDEX, this.turnKi.get());
                this.turnMotor.config_kD(SLOT_INDEX, this.turnKd.get());
            }
        }

        updateDrivePosition();
        updateTurnPosition();
    }

    /**
     * 
     */
    private void updateDrivePosition() {
        this.drivePositionDegrees = Conversions.toDegrees(this.driveMotor.getSelectedSensorPosition(), MK4I_L2.DRIVE_GEAR_RATIO);
        this.driveDistanceMeters = Conversions.toMeters(this.driveMotor.getSelectedSensorPosition(), MK4I_L2.DRIVE_GEAR_RATIO);
        this.driveVelocityMetersPerSecond = Conversions.toMPS(this.driveMotor.getSelectedSensorVelocity(), MK4I_L2.DRIVE_GEAR_RATIO);
    
        this.driveAppliedPercentage = this.driveMotor.getMotorOutputPercent();
        this.driveCurrentAmps = new double[] { this.driveMotor.getStatorCurrent() };
        this.driveTempCelsius = new double[] { this.driveMotor.getTemperature() };
    }

    /**
     * 
     */
    private void updateTurnPosition() {
        this.turnAbsolutePositionDeg = this.encoder.getAbsolutePosition();
        this.turnPositionDeg = Conversions.toDegrees(this.turnMotor.getSelectedSensorPosition(), MK4I_L2.ANGLE_GEAR_RATIO);
        this.turnVelocityRevPerMin = Conversions.toRPM(this.turnMotor.getSelectedSensorVelocity(), MK4I_L2.ANGLE_GEAR_RATIO);

        this.turnAppliedPercentage = this.turnMotor.getMotorOutputPercent();
        this.turnCurrentAmps = new double[] { this.turnMotor.getStatorCurrent()};
        this.turnTempCelsius = new double[] { this.turnMotor.getTemperature()};
    }

    @Override
    public void zeroPIDController() {
        this.driveMotor.config_kP(SLOT_INDEX, 0);
        this.driveMotor.config_kI(SLOT_INDEX, 0);
        this.driveMotor.config_kD(SLOT_INDEX, 0);

        this.turnMotor.config_kP(SLOT_INDEX, 0);
        this.turnMotor.config_kI(SLOT_INDEX, 0);
        this.turnMotor.config_kD(SLOT_INDEX, 0);
    }
}
