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
    private final double DRIVE_KP = 2.0;
    private final double DRIVE_KI = 0.0;
    private final double DRIVE_KD = 0.0;

    /* Simulated Turn Motor PID Values */
    private final double TURN_KP = 2.0;
    private final double TURN_KI = 0.0;
    private final double TURN_KD = 0.0;

    /* Simulated Drive Motor Characterization Values */
    private final double DRIVE_KS = 0.116970;
    private final double DRIVE_KV = 0.133240;
    private final double DRIVE_KA = 0.0;

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

    private boolean isOpenLoop;

    /**
     * 
     */
    public SwerveModuleSimulator(int module_id) {
        super(module_id);

        this.driveMotor = getDriveMotor();
        this.turnMotor = getTurnMotor();

        this.driveController = new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
        this.turnController = new PIDController(turnKp.get(), turnKi.get(), turnKd.get());

        this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
        this.isOpenLoop = false;
    }

    /**
     * 
     */
    @Override
    protected void applyDriveSettings() {
        if (isOpenLoop) {
            this.driveController.reset();
            this.driveAppliedVolts = MathUtil.clamp(this.driveSetpointPercentage * 12.0, -12.0, 12.0);
            this.driveMotor.setInputVoltage(this.driveAppliedVolts);
        }
        else {
            double velocityRadiansPerSecond = this.driveSetpointMPS * (2.0 * Math.PI) / (MK4I_L2.WHEEL_CIRCUMFERENCE);
            double ffOutput = this.feedForward.calculate(velocityRadiansPerSecond); 
            double pidOutput =  this.driveController.calculate(this.driveVelocityMetersPerSecond, velocityRadiansPerSecond);
            
            this.driveAppliedVolts = MathUtil.clamp(ffOutput + pidOutput, -12.0, 12.0);
            this.driveMotor.setInputVoltage(this.driveAppliedVolts);

            Logger.getInstance().recordOutput("SwerveModuleSimulator[" + getModuleId() + "]/Drive FF Output", ffOutput);
            Logger.getInstance().recordOutput("SwerveModuleSimulator[" + getModuleId() + "]/Drive PID Output", pidOutput);
        }
    }

    /**
     * 
     */
    @Override
    protected void applyTurnSettings() {
        double pidOutput = this.turnController.calculate(this.turnRelativePositionRadians, this.turnSetpointDegrees * (Math.PI / 180.0));

        this.turnAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
        this.turnMotor.setInputVoltage(this.turnAppliedVolts);

        Logger.getInstance().recordOutput("SwerveModuleSimulator[" + getModuleId() + "]/Turn PID Output", pidOutput);
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
    }

    @Override
    public void setAnglePosition(double degrees) {
        this.turnSetpointDegrees = degrees;
    }

    @Override
    public void setDriveVelocity(double velocity) {
        this.driveSetpointMPS = velocity;
        this.isOpenLoop = false;
    }

    @Override
    protected void setDrivePercentage(double percentage) {
        this.driveSetpointPercentage = percentage;
        this.isOpenLoop = true;
    }
    
    /**
     * 
     */
    @Override
    public void updatePositions() {
        if (RobotConstants.TUNING_MODE) {
            if (driveKp.hasChanged(hashCode()) || driveKi.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
                this.driveController.setPID(driveKp.get(), driveKi.get(), driveKd.get());
            }
            
            if (turnKp.hasChanged(hashCode()) || turnKi.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
                this.turnController.setPID(turnKp.get(), turnKi.get(), turnKd.get());
            }
        }

        // update the motors
        this.driveMotor.update(RobotConstants.LOOP_PERIOD_SECS);
        this.turnMotor.update(RobotConstants.LOOP_PERIOD_SECS);

        updateDrivePosition();
        updateTurnPosition();

        applyDriveSettings();
        applyTurnSettings();
  
        Logger.getInstance().processInputs("SwerveModuleSimulator", this);
    }

    /**
     * 
     */
    private void updateDrivePosition() {
        this.drivePositionDegrees = this.drivePositionDegrees + (this.driveMotor.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS * (180.0 / Math.PI));
        this.driveDistanceMeters = this.driveDistanceMeters + (this.driveMotor.getAngularVelocityRadPerSec() * RobotConstants.LOOP_PERIOD_SECS * (MK4I_L2.WHEEL_CIRCUMFERENCE / (2.0 * Math.PI)));
        this.driveVelocityMetersPerSecond = this.driveMotor.getAngularVelocityRadPerSec() * (MK4I_L2.WHEEL_CIRCUMFERENCE / (2.0 * Math.PI));

        this.driveAppliedPercentage = this.driveAppliedVolts / 12.0;
        this.driveCurrentAmps = new double[] {Math.abs(this.driveMotor.getCurrentDrawAmps())};
        this.driveTempCelsius = new double[] {};
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
