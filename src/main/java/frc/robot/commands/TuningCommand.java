package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.ProfiledPIDController;
import frc.lib.util.Timer;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;
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
    private final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/kp", PIDConstants.ARM_MODULE_KP);
    private final LoggedTunableNumber armKi = new LoggedTunableNumber("Arm/ki", PIDConstants.ARM_MODULE_KI);
    private final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/kd", PIDConstants.ARM_MODULE_KD);

    /* Tunable PID for individual swerve modules */
    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/kp", PIDConstants.SWERVE_MODULE_DRIVE_KP);
    private final LoggedTunableNumber driveKi = new LoggedTunableNumber("Drive/ki", PIDConstants.SWERVE_MODULE_DRIVE_KI);
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/kd", PIDConstants.SWERVE_MODULE_DRIVE_KD);

    private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Turn/kp", PIDConstants.SWERVE_MODULE_TURN_KP);
    private final LoggedTunableNumber turnKi = new LoggedTunableNumber("Turn/ki", PIDConstants.SWERVE_MODULE_TURN_KI);
    private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Turn/kd", PIDConstants.SWERVE_MODULE_TURN_KD);

    /* Tunable PID used only for Path Planner autonomous mode in the DrivePathCommand */
    private final LoggedTunableNumber xKp = new LoggedTunableNumber("Autonomous/xKp", PIDConstants.SWERVE_DRIVE_X_KP);
    private final LoggedTunableNumber xKi = new LoggedTunableNumber("Autonomous/xKi", PIDConstants.SWERVE_DRIVE_X_KI);
    private final LoggedTunableNumber xKd = new LoggedTunableNumber("Autonomous/xKd", PIDConstants.SWERVE_DRIVE_X_KD);

    private final LoggedTunableNumber yKp = new LoggedTunableNumber("Autonomous/yKp", PIDConstants.SWERVE_DRIVE_Y_KP);
    private final LoggedTunableNumber yKi = new LoggedTunableNumber("Autonomous/yKi", PIDConstants.SWERVE_DRIVE_Y_KI);
    private final LoggedTunableNumber yKd = new LoggedTunableNumber("Autonomous/yKd", PIDConstants.SWERVE_DRIVE_Y_KD);

    private final LoggedTunableNumber omegaKp = new LoggedTunableNumber("Autonomous/omegaKp", PIDConstants.SWERVE_DRIVE_OMEGA_KP);
    private final LoggedTunableNumber omegaKi = new LoggedTunableNumber("Autonomous/omegaKi", PIDConstants.SWERVE_DRIVE_OMEGA_KI);
    private final LoggedTunableNumber omegaKd = new LoggedTunableNumber("Autonomous/omegaKd", PIDConstants.SWERVE_DRIVE_OMEGA_KD);

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
    private final ProfiledPIDController xController = TeleopConstants.xController;
    private final ProfiledPIDController yController = TeleopConstants.yController;
    private final ProfiledPIDController omegaController = TeleopConstants.omegaController;
  
    // Limiters used when driving by controller
    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(TeleopConstants.ROTATION_RATE_LIMIT);

    //
    private final double BACK_AND_FORTH_DISTANCE = 5.0;
    private final Timer timer;

    /**
     * 
     */
    public TuningCommand(SwerveDriveSubsystem swerveDrive, ArmSubsystem armSubsystem, PoseEstimatorSubsystem poseEstimator, XBoxControlBindings controller) {
        this.swerveDrive = swerveDrive;
        this.armSubsystem = armSubsystem;
        this.poseEstimator = poseEstimator;
        this.controller = controller;

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
        // This command should only be used in tuning mode
        if (RobotConstants.TUNING_MODE) {
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
        }

        double xVelocity = this.translateXRateLimiter.calculate(this.controller.translationX().getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(this.controller.translationY().getAsDouble());
        double rVelocity = this.rotationRateLimiter.calculate(this.controller.omega().getAsDouble());
        Rotation2d angle = this.poseEstimator.getCurrentPose().getRotation();
        
        // Drive with controller if there is input
        if (xVelocity != 0 || yVelocity != 0 || rVelocity != 0) {
            handleDriveWithController(xVelocity, yVelocity, rVelocity, angle);
        }
        else if (this.driveBackAndForth.get() || this.driveSideToSide.get() || this.alternateRotation.get()) {
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
            this.timer.stop();
        }
    }

    @Override
    public void initialize() {
        resetPIDControllers();
        // this.poseEstimator.resetPose(new Pose2d(0, 0, new Rotation2d()));

        this.xController.setTolerance(0.02);
        this.yController.setTolerance(0.02);
        this.omegaController.setTolerance(Units.degreesToRadians(1));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

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
