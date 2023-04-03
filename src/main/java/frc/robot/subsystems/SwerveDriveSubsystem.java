package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ProfiledPIDController;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.modules.gyro.GyroModule;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final GyroModule gyro;

    /* These are used only for Path Planner autonomous mode in the DrivePathCommand */
    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;

    /* Tunable PID used only for Path Planner autonomous mode in the DrivePathCommand */
    private final LoggedTunableNumber xKp = new LoggedTunableNumber("PathPlanner/XKp", 2.0);
    private final LoggedTunableNumber xKi = new LoggedTunableNumber("PathPlanner/XKi", 0.0);
    private final LoggedTunableNumber xKd = new LoggedTunableNumber("PathPlanner/XKd", 0.0);

    private final LoggedTunableNumber yKp = new LoggedTunableNumber("PathPlanner/YKp", 2.0);
    private final LoggedTunableNumber yKi = new LoggedTunableNumber("PathPlanner/YKi", 0.0);
    private final LoggedTunableNumber yKd = new LoggedTunableNumber("PathPlanner/YKd", 0.0);

    private final LoggedTunableNumber omegaKp = new LoggedTunableNumber("PathPlanner/OmegaKp", 1.5);
    private final LoggedTunableNumber omegaKi = new LoggedTunableNumber("PathPlanner/OmegaKi", 0.0);
    private final LoggedTunableNumber omegaKd = new LoggedTunableNumber("PathPlanner/OmegaKd", 0.0);

    private ChassisSpeeds desiredChassisSpeeds;
    private double gyroOffset;
    private boolean isFieldOriented;
    private boolean isOpenLoop;

    /**
     * 
     */
    public SwerveDriveSubsystem(SwerveModule[] swerveModules, SwerveDriveKinematics swerveDriveKinematics, GyroModule gyro) {
        this.swerveModules = swerveModules;
        this.swerveDriveKinematics = swerveDriveKinematics;

        this.gyro = gyro;
        zeroGyroscope();
        
        this.isFieldOriented = false;
        this.isOpenLoop = false;
    }
    
    /**
     * 
     */
    public void disableFieldOriented() { setIsFieldOriented(false); }
    public void enableFieldOriented () { setIsFieldOriented(true ); }

    /**
     * 
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        this.desiredChassisSpeeds = chassisSpeeds;
        this.isOpenLoop = isOpenLoop;
    }

    /**
     * 
     */
    public void drive(double xVelocity, double yVelocity, double rVelocity, Rotation2d angle, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = null;

        if (isFieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rVelocity, angle);
        } else {
            chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rVelocity);
        }

        drive(chassisSpeeds, isOpenLoop);
    }

    /**
     * Gets the actual chassis speeds
     * @return actual chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return this.swerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current drivetrain position, as reported by the modules themselves.
     * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
     */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(this.swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
    }
    
    /**
     * Gets the current drivetrain state (velocity, and angle), as reported by the modules themselves.
     * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
     */
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(this.swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
    }

    /**
     * 
     */
    public Rotation2d getRotation() {
        return this.gyro.isConnected() ? Rotation2d.fromDegrees((this.gyro.getPositionDeg() + this.gyroOffset)) : null;
    }

    @Override
    public void periodic() {
        if (RobotConstants.TUNING_MODE) {
            if (xKp.hasChanged(hashCode()) || xKi.hasChanged(hashCode()) || xKd.hasChanged(hashCode())) {
                this.xController.setPID(xKp.get(), xKi.get(), xKd.get());
            }
            
            if (yKp.hasChanged(hashCode()) || yKi.hasChanged(hashCode()) || yKd.hasChanged(hashCode())) {
                this.yController.setPID(yKp.get(), yKi.get(), yKd.get());
            }
            
            if (omegaKp.hasChanged(hashCode()) || omegaKi.hasChanged(hashCode()) || omegaKd.hasChanged(hashCode())) {
                this.omegaController.setPID(omegaKp.get(), omegaKi.get(), omegaKd.get());
            }
        }

        // Set the swerve module states
        if (this.desiredChassisSpeeds != null) {
            SwerveModuleState[] desiredStates = this.swerveDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
            setModuleStates(desiredStates, this.isOpenLoop, false);
        }

        // update and log the swerve module position
        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.updatePositions();
        }

        // log poses, 3D geometry, and swerve module states, gyro offset
        Logger.getInstance().recordOutput("Gyro/isConnected", this.gyro.isConnected());
        Logger.getInstance().recordOutput("Gyro/PositionDeg", this.gyro.getPositionDeg());
        Logger.getInstance().recordOutput("Gyro/VelocityDegPerSec", this.gyro.getVelocityDegPerSec());
        Logger.getInstance().recordOutput("Gyro/Rotation", this.gyro.isConnected() ? getRotation().getDegrees() : 0);
        Logger.getInstance().recordOutput("Gyro/gyroOffset", this.gyroOffset);
        Logger.getInstance().recordOutput("Field Relative", this.isFieldOriented);
        Logger.getInstance().recordOutput("SwerveModuleStates", getModuleStates());
        Logger.getInstance().recordOutput("SwerveDrive/Desired xSpeed", this.desiredChassisSpeeds != null ? this.desiredChassisSpeeds.vxMetersPerSecond : 0);
        Logger.getInstance().recordOutput("SwerveDrive/Actual xSpeed", getChassisSpeeds().vxMetersPerSecond);

        // Always reset desiredChassisSpeeds to null to prevent latching to the last state (aka motor safety)!!
        this.desiredChassisSpeeds = null;
    }

    /**
     * Reseeds the Talon FX steer motors from their CANCoder absolute position. Workaround for "dead wheel"
     */
    public void reseedSteerMotorOffsets() {
        Arrays.stream(this.swerveModules).forEach(SwerveModule::reseedSteerMotorOffset);
    }
    
    /**
     * Sets the rotation of the robot to the specified value. This method should only be invoked when
     * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
     * facing away from the driver station; CCW is positive.
     *
     * @param expectedYaw the rotation of the robot (in degrees)
     */
    public void setGyroOffset(double expectedYaw) {
        // There is a delay between setting the yaw on the Pigeon and that change
        //      taking effect. As a result, it is recommended to never set the yaw and
        //      adjust the local offset instead.
        if (this.gyro.isConnected()) {
            this.gyroOffset = expectedYaw - this.gyro.getPositionDeg();
        } else {
            this.gyroOffset = 0;
        }
    }

    /**
     * Sets each of the swerve modules based on the specified corresponding swerve module state.
     * Incorporates the configured feedforward when setting each swerve module. The order of the
     * states in the array must be front left, front right, back left, back right.
     *
     * <p>This method is invoked by the DrivePathCommand autonomous command.
     *
     * @param states the specified swerve module state for each swerve module
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);

        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.setDesiredState(states[swerveModule.getModuleId()], false, false);
        }
    }

    /**
     * Sets the states of the modules.
     * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
     */
    public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop, boolean forceAngle) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND);

        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.setDesiredState(states[swerveModule.getModuleId()], isOpenLoop, forceAngle);
        }
    }

    /**
     * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
     * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
     * useful when shooting.
     */
    public void setXStance() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = this.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(0.0, 0.0));
        states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(DriveTrainConstants.TRACK_WIDTH_METERS / DriveTrainConstants.WHEEL_BASE_METERS));
        states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(DriveTrainConstants.TRACK_WIDTH_METERS / DriveTrainConstants.WHEEL_BASE_METERS));
        states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(DriveTrainConstants.TRACK_WIDTH_METERS / DriveTrainConstants.WHEEL_BASE_METERS));
        states[3].angle = new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(DriveTrainConstants.TRACK_WIDTH_METERS / DriveTrainConstants.WHEEL_BASE_METERS));

        setModuleStates(states, true, true);
    }

    /**
     * 
     */
    public void stop() {
        drive(new ChassisSpeeds(), false);
    }

    /**
     * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
     * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
     * sufficiently different to make field-relative driving difficult. The robot needs to be
     * positioned facing away from the driver, ideally aligned to a field wall before this method is
     * invoked.
     */
    public void zeroGyroscope() {
        setGyroOffset(0.0);
    }

    /** Getters and Setters */
    public ProfiledPIDController getXController() { return this.xController; }
    public ProfiledPIDController getYController() { return this.yController; }
    public ProfiledPIDController getOmegaController() { return this.omegaController; }

    public boolean isFieldOriented() { return this.isFieldOriented; }
    public GyroModule getGyro() { return this.gyro; }
    public SwerveDriveKinematics getKinematics() { return this.swerveDriveKinematics; }

    public void setIsFieldOriented(boolean isFieldOriented) { this.isFieldOriented = isFieldOriented; }
}
