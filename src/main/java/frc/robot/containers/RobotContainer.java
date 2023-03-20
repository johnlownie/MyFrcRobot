package frc.robot.containers;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.FieldConstants;
import frc.robot.autonomous.AutonomousBuilder;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrive;

abstract public class RobotContainer {
   /* Autonomous */
    protected AutonomousBuilder autonomousBuilder;

    /* Commands */
    protected TeleopDriveCommand teleopDriveCommand;

    /* Controllers */
    protected final XBoxControlBindings driverController;

    /* Subsystems */
    protected SwerveDrive swerveDrive;
    protected PoseEstimator poseEstimator;

    /**
     * 
     */
    public RobotContainer() {
        this.driverController = new XBoxControlBindings();
    }

    /**
     * 
     */
    protected void configureButtonBindings() {
        // switch from field relative to field oriented
        this.driverController.driveType().ifPresent(trigger -> trigger.toggleOnTrue(
            either(
                runOnce(this.swerveDrive::disableFieldOriented, this.swerveDrive),
                runOnce(this.swerveDrive::enableFieldOriented, this.swerveDrive),
                this.swerveDrive::isFieldOriented)
        ));

        // reset the robot pose
        this.driverController.resetPose().ifPresent(trigger -> trigger.onTrue(runOnce(this.poseEstimator::resetFieldPosition)));

        // Start button reseeds the steer motors to fix dead wheel
        this.driverController.reseedSteerMotors()
            .ifPresent(trigger -> trigger.onTrue(this.swerveDrive.runOnce(this.swerveDrive::reseedSteerMotorOffsets)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));

        // X-Stance Pose
        this.driverController.xStance().ifPresent(trigger -> trigger.onTrue(
            run(this.swerveDrive::setXStance)
            .until(this.driverController.driverWantsControl())
        ));
    }

    /**
     * 
     */
    protected void configureDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        this.autonomousBuilder.addDashboardWidgets(tab);
    }

    /**
     * 
     */
    public Command getAutonomousCommand() {
        return this.autonomousBuilder.getAutonomousCommand();
    }
        
    /**
     * Called when the alliance reported by the driverstation/FMS changes.
     * @param alliance new alliance value
     */
    public void onAllianceChanged(Alliance alliance, int location) {
        int allianceStation = alliance.name().equalsIgnoreCase("blue") ? 0 : 1;
        // this.poseEstimator.setAlliance(alliance, this.alliancePoses[allianceStation][location - 1]);
        this.poseEstimator.setCurrentPose(FieldConstants.ALLIANCE_POSES[allianceStation][location - 1]);
    }

    /**
     * 
     */
    protected void setCommands() {
        this.teleopDriveCommand = new TeleopDriveCommand(
            this.swerveDrive,
            () -> this.poseEstimator.getCurrentPose().getRotation(),
            this.driverController.translationX(),
            this.driverController.translationY(),
            this.driverController.omega()
        );
        
        this.swerveDrive.setDefaultCommand(this.teleopDriveCommand);
    }
}
