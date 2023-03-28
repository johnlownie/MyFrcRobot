package frc.robot.modules.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SwerveModuleConstants.MK4I_L2;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class SwerveModuleSimulator extends SwerveModule {
    /* Simulated Drive Motor PID Values */
    private static final double DRIVE_KP = 2.0;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0;

    /* Simulated Turn Motor PID Values */
    private static final double TURN_KP = 12.0;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.0;

    /* Simulated Drive Motor Characterization Values */
    private static final double DRIVE_KS = 0.116970;
    private static final double DRIVE_KV = 0.133240;
    private static final double DRIVE_KA = 0.0;

    /* Tunable PID */
    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp", DRIVE_KP);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drive/DriveKi", DRIVE_KI);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd", DRIVE_KD);

    private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/TurnKp", TURN_KP);
    private final LoggedTunableNumber turnKi = new LoggedTunableNumber("Drive/TurnKi", TURN_KI);
    private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd", TURN_KD);

    /* Motors */
    private FlywheelSim driveMotor;
    private FlywheelSim turnMotor;

    private PIDController driveController;
    private PIDController turnController;
    
    private SimpleMotorFeedforward feedForward;
    private double turnAbsolutePositionRadians = Math.random() * 2.0 * Math.PI;
    private double turnRelativePositionRadians = 0.0;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id) {
        super(module_id);

        this.driveMotor = getDriveMotor();
        this.driveController = new PIDController(driveKp.get(), driveKi.get(), driveKd.get());

        this.turnMotor = getTurnMotor();
        this.turnController = new PIDController(turnKp.get(), turnKi.get(), turnKd.get());

        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
    }

    /**
     * 
     */
    private FlywheelSim getDriveMotor() {
        FlywheelSim motor = new FlywheelSim(DCMotor.getFalcon500(1), SwerveModuleConstants.MK4I_L2.DRIVE_GEAR_RATIO, 0.025);

        return motor;
    }

    /**
     * 
     */
    private FlywheelSim getTurnMotor() {
        FlywheelSim motor = new FlywheelSim(DCMotor.getFalcon500(1), SwerveModuleConstants.MK4I_L2.ANGLE_GEAR_RATIO, 0.004096955);

        return motor;
    }

    /**
     * Reseeds to Talon FX motor offset from the CANCoder. Workaround for "dead wheel"
     */
    @Override
    public void reseedSteerMotorOffset() {
        // this.turnController.configMotorOffset(false);
    }

    /**
     * 
     */
    @Override
    protected void setDrivePercentage(double percentage) {
        this.driveController.reset();
        this.driveAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
        this.driveMotor.setInputVoltage(driveAppliedVolts);
    }

    /**
     * 
     */
    @Override
    protected void setDriveVelocity(double velocity) {
        double velocityRadiansPerSecond = velocity * (2.0 * Math.PI) / (MK4I_L2.WHEEL_CIRCUMFERENCE);
        double driveAppliedVolts = this.feedForward.calculate(velocityRadiansPerSecond) + this.driveController.calculate(this.driveVelocityMetersPerSecond, velocityRadiansPerSecond);
        driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);

        this.driveAppliedVolts = driveAppliedVolts;
        this.driveMotor.setInputVoltage(driveAppliedVolts);
    }

    /**
     * 
     */
    @Override
    protected void setTurnRotation() {
        double turnAppliedVolts = this.turnController.calculate(this.turnRelativePositionRadians, this.angleSetpointDegrees * (Math.PI / 180.0));
        turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);

        this.turnAppliedVolts = turnAppliedVolts;
        this.turnMotor.setInputVoltage(turnAppliedVolts);
    }

    /**
     * 
     */
    private void updateDrivePosition() {
        this.drivePositionDegrees += (this.driveMotor.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS * (180.0 / Math.PI));
        this.driveDistanceMeters += (this.driveMotor.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS * (MK4I_L2.WHEEL_CIRCUMFERENCE / (2.0 * Math.PI)));
        this.driveVelocityMetersPerSecond = this.driveMotor.getAngularVelocityRadPerSec() * (MK4I_L2.WHEEL_CIRCUMFERENCE / (2.0 * Math.PI));

        this.driveAppliedPercentage = this.driveAppliedVolts / 12.0;
        this.driveCurrentAmps = new double[] {Math.abs(this.driveMotor.getCurrentDrawAmps())};
        this.driveTempCelsius = new double[] {};
    }
      
    @Override
    public void updatePositions() {
        // update the motors
        this.driveMotor.update(RobotConstants.LOOP_PERIOD_SECS);
        this.turnMotor.update(RobotConstants.LOOP_PERIOD_SECS);

        // update the inputs that will be logged
        updateDrivePosition();
        updateTurnPosition();
        setTurnRotation();

        // Update tunable numbers
        if (driveKp.hasChanged(hashCode()) || driveKi.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
            this.driveController.setPID(driveKp.get(), driveKi.get(), driveKd.get());
        }

        if (turnKp.hasChanged(hashCode()) || turnKi.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
            this.turnController.setPID(turnKp.get(), turnKi.get(), turnKd.get());
        }
  
        Logger.getInstance().processInputs("SwerveModuleSimulator", this);
    }

    /**
     * 
     */
    private void updateTurnPosition() {
        double angleDiffRadians = this.turnMotor.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS;
        this.turnRelativePositionRadians += angleDiffRadians;
        this.turnAbsolutePositionRadians += angleDiffRadians;

        while (this.turnAbsolutePositionRadians < 0) {
            this.turnAbsolutePositionRadians += 2.0 * Math.PI;
        }

        while (turnAbsolutePositionRadians > 2.0 * Math.PI) {
            this.turnAbsolutePositionRadians -= 2.0 * Math.PI;
        }

        this.turnAbsolutePositionDeg = this.turnAbsolutePositionRadians * (180.0 / Math.PI);
        this.turnPositionDeg = this.turnRelativePositionRadians * (180.0 / Math.PI);
        this.turnVelocityRevPerMin = this.turnMotor.getAngularVelocityRadPerSec() * (60.0 / (2.0 * Math.PI));

        this.turnAppliedPercentage = this.turnAppliedVolts / 12.0;
        this.turnCurrentAmps = new double[] {Math.abs(this.turnMotor.getCurrentDrawAmps())};
        this.turnTempCelsius = new double[] {};
    }
}
