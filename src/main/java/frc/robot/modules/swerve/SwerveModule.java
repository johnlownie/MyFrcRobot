package frc.robot.modules.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * 
 */
@AutoLog
abstract public class SwerveModule implements LoggableInputs {
    private int module_id;
    private double last_angle;

    protected double driveSetpointMPS = 0.0;
    protected double driveSetpointPercentage = 0.0;
    protected double turnSetpointDegrees = 0.0;

    /* Calculated input values */
    double driveAppliedPercentage = 0.0;
    double driveAppliedVolts = 0.0;
    double driveDistanceMeters = 0.0;
    double drivePositionDegrees = 0.0;
    double driveVelocityMetersPerSecond = 0.0;
    double[] driveCurrentAmps = new double[] {};
    double[] driveTempCelsius = new double[] {};

    double turnAbsolutePositionDeg = 0.0;
    double turnAppliedPercentage = 0.0;
    double turnAppliedVolts = 0.0;
    double turnPositionDeg = 0.0;
    double turnVelocityRevPerMin = 0.0;
    double[] turnCurrentAmps = new double[] {};
    double[] turnTempCelsius = new double[] {};

    /**
     * 
     */
    public SwerveModule(int module_id) {
        this.module_id = module_id;
        this.last_angle = getState().angle.getDegrees();
    }

    /**
     *
     */
    public int getModuleId() { return this.module_id; }

    /**
     * 
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getStatePosition(), getStateRotation());
    }

    /**
     * Get the current state of this swerve module.
     *
     * @return the current state of this swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.driveVelocityMetersPerSecond, Rotation2d.fromDegrees(this.turnPositionDeg));
    }

    /**
     * 
     */
    public double getStatePosition() {
        return this.driveDistanceMeters;
    }

    /**
     * 
     */
    public Rotation2d getStateRotation() {
        return Rotation2d.fromDegrees(this.turnPositionDeg);
    }

    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    public void setDesiredState(SwerveModuleState moduleState, boolean isOpenLoop, boolean forceAngle) {
        moduleState = CTREModuleState.optimize(moduleState, getState().angle);

        if (isOpenLoop) {
            double percentOuput = moduleState.speedMetersPerSecond / SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND;
            setDrivePercentage(percentOuput);
        }
        else {
            setDriveVelocity(moduleState.speedMetersPerSecond);
        }

        double angle = 0.0;
        if (!forceAngle && Math.abs(moduleState.speedMetersPerSecond) <= (SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) {
            angle = this.last_angle;
        }
        else {
            angle = moduleState.angle.getDegrees();
        }

        this.last_angle = angle;
        setAnglePosition(this.last_angle);
    }

    abstract protected void applyDriveSettings();
    abstract protected void applyTurnSettings();

    abstract public void setAnglePosition(double degrees);
    abstract protected void setDrivePercentage(double percentage);
    abstract protected void setDriveVelocity(double velocity);

    abstract public void reseedSteerMotorOffset();
    abstract public void resetPIDController();
    abstract public void updatePositions();
    abstract public void zeroPIDController();

    /**
     * Logging
     */
    @Override
    public void fromLog(LogTable table) {
        this.driveAppliedPercentage = table.getDouble("DriveAppliedPercentage", this.driveAppliedPercentage);
        this.driveAppliedVolts = table.getDouble("DriveAppliedVolts", this.driveAppliedVolts);
        this.driveDistanceMeters = table.getDouble("DriveDistanceMeters", this.driveDistanceMeters);
        this.drivePositionDegrees = table.getDouble("DrivePositionDeg", this.drivePositionDegrees);
        this.driveVelocityMetersPerSecond = table.getDouble("DriveVelocityMetersPerSec", this.driveVelocityMetersPerSecond);
        this.driveCurrentAmps = table.getDoubleArray("DriveCurrentAmps", this.driveCurrentAmps);
        this.driveTempCelsius = table.getDoubleArray("DriveTempCelsius", this.driveTempCelsius);

        this.turnAbsolutePositionDeg = table.getDouble("TurnAbsolutePositionDeg", this.turnAbsolutePositionDeg);
        this.turnAppliedPercentage = table.getDouble("TurnAppliedPercentage", this.turnAppliedPercentage);
        this.turnAppliedVolts = table.getDouble("TurnAppliedVolts", this.turnAppliedVolts);
        this.turnPositionDeg = table.getDouble("TurnPositionDeg", this.turnPositionDeg);
        this.turnVelocityRevPerMin = table.getDouble("TurnVelocityRevPerMin", this.turnVelocityRevPerMin);
        this.turnCurrentAmps = table.getDoubleArray("TurnCurrentAmps", this.turnCurrentAmps);
        this.turnTempCelsius = table.getDoubleArray("TurnTempCelsius", this.turnTempCelsius);
    }
  
    @Override
    public void toLog(LogTable table) {
        table.put("DriveSetpointMPS", this.driveSetpointMPS);
        table.put("DriveSetpointPercentage", this.driveSetpointPercentage);
        table.put("DriveAppliedPercentage", this.driveAppliedPercentage);
        table.put("DriveAppliedVolts", this.driveAppliedVolts);
        table.put("DriveDistanceMeters", this.driveDistanceMeters);
        table.put("DrivePositionDeg", this.drivePositionDegrees);
        table.put("DriveVelocityMetersPerSec", this.driveVelocityMetersPerSecond);
        table.put("DriveCurrentAmps", this.driveCurrentAmps);
        table.put("DriveTempCelsius", this.driveTempCelsius);

        table.put("TurnSetpointDegrees", this.turnSetpointDegrees);
        table.put("TurnAbsolutePositionDeg", this.turnAbsolutePositionDeg);
        table.put("TurnAppliedPercentage", this.turnAppliedPercentage);
        table.put("TurnAppliedVolts", this.turnAppliedVolts);
        table.put("TurnPositionDeg", this.turnPositionDeg);
        table.put("TurnVelocityRevPerMin", this.turnVelocityRevPerMin);
        table.put("TurnCurrentAmps", this.turnCurrentAmps);
        table.put("TurnTempCelsius", this.turnTempCelsius);
    }
}
