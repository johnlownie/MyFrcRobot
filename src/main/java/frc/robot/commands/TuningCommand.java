package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Timer;
import frc.robot.Robot;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LoggedTunableNumber;

/**
 * 
 */
public class TuningCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final XBoxControlBindings controller;

    /* Tunable PID for arm module */
    private LoggedTunableNumber armKp;
    private LoggedTunableNumber armKi;
    private LoggedTunableNumber armKd;

    /* Tunable PID for individual swerve modules */
    private LoggedTunableNumber driveKp;
    private LoggedTunableNumber driveKi;
    private LoggedTunableNumber driveKd;

    private LoggedTunableNumber turnKp;
    private LoggedTunableNumber turnKi;
    private LoggedTunableNumber turnKd;

    /* Tunable PID used only for Path Planner autonomous mode in the DrivePathCommand */
    private LoggedTunableNumber xKp;
    private LoggedTunableNumber xKi;
    private LoggedTunableNumber xKd;

    private LoggedTunableNumber yKp;
    private LoggedTunableNumber yKi;
    private LoggedTunableNumber yKd;

    private LoggedTunableNumber omegaKp;
    private LoggedTunableNumber omegaKi;
    private LoggedTunableNumber omegaKd;

    // Tunables used to set the robot to a particular Pose2d
    private final LoggedTunableNumber xPosition = new LoggedTunableNumber("Robot/xPosition", 0.0);
    private final LoggedTunableNumber yPosition = new LoggedTunableNumber("Robot/yPosition", 0.0);
    private final LoggedTunableNumber rAngle = new LoggedTunableNumber("Robot/rAngle", 0.0);

    private final LoggedDashboardBoolean driveBackAndForth = new LoggedDashboardBoolean("Drive Back and Forth");
    private final LoggedDashboardBoolean driveSideToSide = new LoggedDashboardBoolean("Drive Side to Side");
    private final LoggedDashboardBoolean alternateRotation = new LoggedDashboardBoolean("Alternate Rotation");

    // Tunable to set the arm subsystem to a particular angle
    private final LoggedTunableNumber desiredAngle = new LoggedTunableNumber("Robot/DesiredArmAngle", 0.0);

    /* These are used only for Path Planner autonomous mode in the DrivePathCommand */
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;
  
    // Limiters used when driving by controller
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(TeleopConstants.ROTATION_RATE_LIMIT);

    //
    private final double BACK_AND_FORTH_DISTANCE = 5.0;
    private final Timer timer;

    /**
     * This command should only be run in robot tuning mode
     */
    public TuningCommand(SwerveDriveSubsystem swerveDrive, ArmSubsystem armSubsystem, PoseEstimatorSubsystem poseEstimator, XBoxControlBindings controller) {
        this.swerveDrive = swerveDrive;
        this.armSubsystem = armSubsystem;
        this.poseEstimator = poseEstimator;
        this.controller = controller;

        double[] driveXPIDs = PIDConstants.getDriveXPIDs();
        double[] driveYPIDs = PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();

        this.xController = new ProfiledPIDController(driveXPIDs[0], driveXPIDs[1], driveXPIDs[2], TeleopConstants.X_CONSTRAINTS);
        this.yController = new ProfiledPIDController(driveYPIDs[0], driveYPIDs[1], driveYPIDs[2], TeleopConstants.Y_CONSTRAINTS);
        this.omegaController = new ProfiledPIDController(driveOmegaPIDs[0], driveOmegaPIDs[1], driveOmegaPIDs[2], TeleopConstants.OMEGA_CONSTRAINTS);

        this.xController.setTolerance(0.02);
        this.yController.setTolerance(0.02);
        this.omegaController.setTolerance(Units.degreesToRadians(1));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.timer = new Timer();

        addRequirements(swerveDrive, armSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        if (this.armKp.hasChanged(hashCode()) || this.armKi.hasChanged(hashCode()) || this.armKd.hasChanged(hashCode())) {
            this.armSubsystem.updatePID(this.armKp.get(), this.armKi.get(), this.armKd.get());
        }

        if (this.driveKp.hasChanged(hashCode()) || this.driveKi.hasChanged(hashCode()) || this.driveKd.hasChanged(hashCode())) {
            this.swerveDrive.updateDrivePID(this.driveKp.get(), this.driveKi.get(), this.driveKd.get());
        }
        
        if (this.turnKp.hasChanged(hashCode()) || this.turnKi.hasChanged(hashCode()) || this.turnKd.hasChanged(hashCode())) {
            this.swerveDrive.updateTurnPID(this.turnKp.get(), this.turnKi.get(), this.turnKd.get());
        }
        
        if (this.xKp.hasChanged(hashCode()) || this.xKi.hasChanged(hashCode()) || this.xKd.hasChanged(hashCode())) {
            this.xController.setPID(this.xKp.get(), this.xKi.get(), this.xKd.get());
        }
        
        if (this.yKp.hasChanged(hashCode()) || this.yKi.hasChanged(hashCode()) || this.yKd.hasChanged(hashCode())) {
            this.yController.setPID(this.yKp.get(), this.yKi.get(), this.yKd.get());
        }
        
        if (this.omegaKp.hasChanged(hashCode()) || this.omegaKi.hasChanged(hashCode()) || this.omegaKd.hasChanged(hashCode())) {
            this.omegaController.setPID(this.omegaKp.get(), this.omegaKi.get(), this.omegaKd.get());
        }
        
        if (this.xPosition.hasChanged(hashCode())) {
            this.xController.setGoal(this.xPosition.get());
        }
        
        if (this.yPosition.hasChanged(hashCode())) {
            this.yController.setGoal(this.yPosition.get());
        }
        
        if (this.rAngle.hasChanged(hashCode())) {
            this.omegaController.setGoal(Units.degreesToRadians(this.rAngle.get()));
        }
        
        if (this.desiredAngle.hasChanged(hashCode())) {
            this.armSubsystem.setDesiredAngle(this.desiredAngle.get());
        }

        double xVelocity = this.translateXRateLimiter.calculate(this.controller.translationX().getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(this.controller.translationY().getAsDouble());
        double rVelocity = this.rotationRateLimiter.calculate(this.controller.omega().getAsDouble());
        Rotation2d angle = this.poseEstimator.getCurrentPose().getRotation();
        
        if (this.driveBackAndForth.get() || this.driveSideToSide.get() || this.alternateRotation.get()) {
            handleAutonomousDriving();
        }
        // else if (!isAtGoal()) {
            //     Pose2d robotPose = this.poseEstimator.getCurrentPose();
            
            //     double xSpeed = this.xController.calculate(robotPose.getX());
            //     double ySpeed = this.yController.calculate(robotPose.getY());
            //     double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());
            
            //     if (this.xController.atGoal()) xSpeed = 0;
            //     if (this.yController.atGoal()) ySpeed = 0;
            //     if (this.omegaController.atGoal()) omegaSpeed = 0;
            
            //     this.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()), false);
            // }
        else {
            handleDriveWithController(xVelocity, yVelocity, rVelocity, angle);
            this.timer.stop();
        }
    }

    @Override
    public void initialize() {
        double[] driveXPIDs = PIDConstants.getDriveXPIDs();
        double[] driveYPIDs = PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();

        this.armKp = new LoggedTunableNumber("Arm/kp", PIDConstants.ARM_MODULE_KP);
        this.armKi = new LoggedTunableNumber("Arm/ki", PIDConstants.ARM_MODULE_KI);
        this.armKd = new LoggedTunableNumber("Arm/kd", PIDConstants.ARM_MODULE_KD);

        this.driveKp = new LoggedTunableNumber("Drive/kp", PIDConstants.SWERVE_MODULE_DRIVE_KP);
        this.driveKi = new LoggedTunableNumber("Drive/ki", PIDConstants.SWERVE_MODULE_DRIVE_KI);
        this.driveKd = new LoggedTunableNumber("Drive/kd", PIDConstants.SWERVE_MODULE_DRIVE_KD);

        this.turnKp = new LoggedTunableNumber("Turn/kp", PIDConstants.SWERVE_MODULE_TURN_KP);
        this.turnKi = new LoggedTunableNumber("Turn/ki", PIDConstants.SWERVE_MODULE_TURN_KI);
        this.turnKd = new LoggedTunableNumber("Turn/kd", PIDConstants.SWERVE_MODULE_TURN_KD);

        this.xKp = new LoggedTunableNumber("Autonomous/xKp", driveXPIDs[0]);
        this.xKi = new LoggedTunableNumber("Autonomous/xKi", driveXPIDs[1]);
        this.xKd = new LoggedTunableNumber("Autonomous/xKd", driveXPIDs[2]);
    
        this.yKp = new LoggedTunableNumber("Autonomous/yKp", driveYPIDs[0]);
        this.yKi = new LoggedTunableNumber("Autonomous/yKi", driveYPIDs[1]);
        this.yKd = new LoggedTunableNumber("Autonomous/yKd", driveYPIDs[2]);
    
        this.omegaKp = new LoggedTunableNumber("Autonomous/omegaKp", driveOmegaPIDs[0]);
        this.omegaKi = new LoggedTunableNumber("Autonomous/omegaKi", driveOmegaPIDs[1]);
        this.omegaKd = new LoggedTunableNumber("Autonomous/omegaKd", driveOmegaPIDs[2]);

        resetPIDControllers();
        // this.poseEstimator.resetPose(new Pose2d(0, 0, new Rotation2d()));

        this.xController.setGoal(this.poseEstimator.getCurrentPose().getX());
        this.yController.setGoal(this.poseEstimator.getCurrentPose().getY());
        this.omegaController.setGoal(this.poseEstimator.getCurrentPose().getRotation().getRadians());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }
    
    /**
     * 
     */
    private void handleAutonomousDriving() {
        Logger.recordOutput("Commands/TuningCommand/GoalPosition", this.xController.getGoal().position);
        Logger.recordOutput("Commands/TuningCommand/GoalVelocity", this.xController.getGoal().velocity);

        if (isAtGoal()) {
            this.swerveDrive.stop();

            if (!this.timer.isRunning()) {
                this.timer.reset();
                this.timer.start();
            }
            else if (this.timer.isRunning() && this.timer.hasElapsed(5.0)) {
                this.timer.stop();

                if (this.driveBackAndForth.get()) {
                    double xCurrent = this.poseEstimator.getCurrentPose().getX();
                    double xGoal = xCurrent >= 2.5 ? 0 : BACK_AND_FORTH_DISTANCE;
                    this.xController.setGoal(xGoal);
                }
            }
        }
        else {
            Pose2d robotPose = this.poseEstimator.getCurrentPose();

            double xSpeed = this.xController.calculate(robotPose.getX());
            double ySpeed = this.yController.calculate(robotPose.getY());
            double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());

            Logger.recordOutput("Commands/TuningCommand/xSpeed", xSpeed);
            Logger.recordOutput("Commands/TuningCommand/ySpeed", ySpeed);
            Logger.recordOutput("Commands/TuningCommand/omegaSpeed", omegaSpeed);
    
            if (this.xController.atGoal()) xSpeed = 0;
            if (this.yController.atGoal()) ySpeed = 0;
            if (this.omegaController.atGoal()) omegaSpeed = 0;
    
            this.swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()), false);
        }
    }

    /**
     * 
     */
    private void handleDriveWithController(double xVelocity, double yVelocity, double rVelocity, Rotation2d angle) {
        if (this.driveBackAndForth.get() || this.driveSideToSide.get() || this.alternateRotation.get()) {
            this.driveBackAndForth.set(false);
            this.driveSideToSide.set(false);
            this.alternateRotation.set(false);
        }

        this.swerveDrive.drive(xVelocity, yVelocity, rVelocity, angle, true);

        Logger.recordOutput("Commands/TuningCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/TuningCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/TuningCommand/rVelocity", rVelocity);
        Logger.recordOutput("Commands/TuningCommand/angle", angle.getDegrees());
    }

    /**
     * 
     */
    private boolean isAtGoal() {
        return this.xController.atGoal() && this.yController.atGoal() && this.omegaController.atGoal();
    }

    /**
     * 
     */
    private void resetPIDControllers() {
        Pose2d robotPose = this.poseEstimator.getCurrentPose();

        this.xController.reset(robotPose.getX());
        this.yController.reset(robotPose.getY());
        this.omegaController.reset(robotPose.getRotation().getRadians());
    }
}
