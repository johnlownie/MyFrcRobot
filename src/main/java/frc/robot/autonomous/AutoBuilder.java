package frc.robot.autonomous;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {
    /* Subsystems */
    private final PoseEstimatorSubsystem poseEstimatorSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    /* Autonomous */
    SendableChooser<Command> autonomousChooser;

    /**
     * 
     */
    public AutoBuilder(PoseEstimatorSubsystem poseEstimatorSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    /**
     * 
     */
    public void configureAutonomous() {
        AutoBuilder.configureHolonomic(
            this.poseEstimatorSubsystem::getCurrentPose,
            this.poseEstimatorSubsystem::resetPose,
            this.swerveDriveSubsystem::getChassisSpeeds,
            this.swerveDriveSubsystem::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0),
                SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND,
                DriveTrainConstants.TRACK_WIDTH_METERS,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            this.swerveDriveSubsystem
        );

        // Setup the chooser
        this.autonomousChooser = AutoBuilder.buildAutoChooser();
        this.autonomousChooser.addOption("Shoot and Move Away", getShootAndMoveAway());
        this.autonomousChooser.addOption("Blue 1 - Drive First", getBlue1DriveFirst());

        // Add chooser to the shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        tab.add("Autonomous", this.autonomousChooser).withSize(2, 1).withPosition(0, 0);
    }

    /**
     * 
     */
    public SendableChooser<Command> getAutonomousChooser() {
        return this.autonomousChooser;
    }

    /**
     * 
     */
    private Command getShootAndMoveAway() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Blue3 - Center5");

        if (path == null) {
            return Commands.print("*** Failed to build ShootAndMoveAway");
        }

        Logger.recordOutput("Autonomous/Path", path.getPreviewStartingHolonomicPose());

        Command command = Commands.sequence(
            Commands.print("*** Starting ShootAndMoveAway ***"),

            new InstantCommand(() -> {
                this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
            }),
            new WaitUntilCommand(this.armSubsystem::isAtAngle),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_SPEAKER);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.PAUSE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(path)
            ),

            Commands.print("*** Finished ShootAndMoveAway ***")
        );

        return command;
    }

    /**
     * 
     */
    private Command getBlue1DriveFirst() {
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Blue 1 - Drive First");

        if (pathGroup == null) {
            return Commands.print("*** Failed to build Blue1DriveFirst");
        }

        Command command = Commands.sequence(
            Commands.print("*** Starting Blue1DriveFirst ***"),

            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.PAUSE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(0))
            ),
            new WaitUntilCommand(this.intakeSubsystem::hasNote),
            new ParallelCommandGroup(
                AutoBuilder.followPath(pathGroup.get(1)),
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                }),
                new WaitUntilCommand(this.armSubsystem::isAtAngle)
            ),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_SPEAKER);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            new WaitUntilCommand(this.intakeSubsystem::hasNote),
            new ParallelCommandGroup(
                AutoBuilder.followPath(pathGroup.get(3)),
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                }),
                new WaitUntilCommand(this.armSubsystem::isAtAngle)
            ),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT_SPEAKER);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(4))
            ),

            Commands.print("*** Finished Blue1DriveFirst ***")
        );

        return command;
    }
}
