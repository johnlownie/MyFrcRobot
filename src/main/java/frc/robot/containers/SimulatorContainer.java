package frc.robot.containers;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.autonomous.AutonomousBuilder;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DriveToTagCommand;
import frc.robot.modules.gyro.GyroModuleSimulator;
import frc.robot.modules.swerve.SwerveModuleSimulator;
import frc.robot.modules.vision.VisionModuleSimulator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrive;

/**
 * 
 */
public class SimulatorContainer extends RobotContainer {
    /* Modules */
    private final SwerveModuleSimulator[] swerveModules;
    private final GyroModuleSimulator gyro;
    private final VisionModuleSimulator visionModule;

    /* Commands */
    private DriveToTagCommand driveToTagLeftCommand;
    private DriveToTagCommand driveToTagRightCommand;
    
    private DriveToPoseCommand driveToPoleLeftCommand;
    private DriveToPoseCommand driveToPoleRightCommand;

    /**
     * 
     */
    public SimulatorContainer() {
        super();
        
        SwerveModuleSimulator frontLeftModule = new SwerveModuleSimulator(0);
        SwerveModuleSimulator frontRightModule = new SwerveModuleSimulator(1);
        SwerveModuleSimulator rearLeftModule = new SwerveModuleSimulator(2);
        SwerveModuleSimulator rearRightModule = new SwerveModuleSimulator(3);
        
        this.swerveModules = new SwerveModuleSimulator[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule};
        this.gyro = new GyroModuleSimulator();
        
        this.swerveDrive = new SwerveDrive(this.swerveModules, DriveTrainConstants.SWERVE_DRIVE_KINEMATICS, this.gyro);
        this.visionModule = new VisionModuleSimulator(AprilTagFields.k2023ChargedUp);
        this.poseEstimator = new PoseEstimator(this.swerveDrive::getModulePositions, this.swerveDrive::getRotation, this.visionModule);
        
        this.autonomousBuilder = new AutonomousBuilder(this.swerveDrive, this.poseEstimator);

        setCommands();

        configureButtonBindings();
        configureDashboard();
    }

    @Override
    protected void configureButtonBindings() {
        super.configureButtonBindings();

        // POV up for field oriented drive
        this.driverController.driveTeleop().ifPresent(
            trigger -> trigger.onTrue(runOnce(() -> this.swerveDrive.setDefaultCommand(this.teleopDriveCommand))
                .andThen(new ScheduleCommand(this.teleopDriveCommand))));

        // drive to tag and strafe
        this.driverController.driveToPoleLeft().ifPresent(
            trigger -> trigger.onTrue(
                this.driveToTagLeftCommand
                    .andThen(
                        this.driveToPoleLeftCommand
                        .until(this.driverController.driverWantsControl())
                    )
                    .until(this.driverController.driverWantsControl())
            )
        );

        // drive to tag and strafe
        this.driverController.driveToPoleRight().ifPresent(
            trigger -> trigger.onTrue(
                this.driveToTagRightCommand
                    .andThen(
                        this.driveToPoleRightCommand
                        .until(this.driverController.driverWantsControl())
                    )
                    .until(this.driverController.driverWantsControl())
            )
        );
    }

    @Override
    protected void setCommands() {
        super.setCommands();
        
        this.driveToTagLeftCommand = new DriveToTagCommand(this.swerveDrive, this.visionModule.getCamera(), this.poseEstimator::getCurrentPose);
        this.driveToPoleLeftCommand = new DriveToPoseCommand(this.swerveDrive, this.poseEstimator::getCurrentPose, 0.0, -0.55, 0.0);

        this.driveToTagRightCommand = new DriveToTagCommand(this.swerveDrive, this.visionModule.getCamera(), this.poseEstimator::getCurrentPose);
        this.driveToPoleRightCommand = new DriveToPoseCommand(this.swerveDrive, this.poseEstimator::getCurrentPose, 0.0, 0.55, 0.0);
    }
}
