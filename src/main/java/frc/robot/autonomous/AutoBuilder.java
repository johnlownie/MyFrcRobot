package frc.robot.autonomous;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.TargetAndGrabNoteCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {
    /* Subsystems */
    private final PoseEstimatorSubsystem poseEstimatorSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PathConstraints constraints = new PathConstraints(
        SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND,
        4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    /* Autonomous */
    private LoggedDashboardChooser<Command> autonomousChooser;
    /**
     * 
     */
    public AutoBuilder(PoseEstimatorSubsystem poseEstimatorSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
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
                new PIDConstants(TeleopConstants.xController.getP(), TeleopConstants.xController.getI(), TeleopConstants.xController.getD()),
                new PIDConstants(TeleopConstants.omegaController.getP(), TeleopConstants.omegaController.getI(), TeleopConstants.omegaController.getD()),
                SwerveModuleConstants.MAX_VELOCITY_METERS_PER_SECOND,
                Math.hypot(DriveTrainConstants.TRACK_WIDTH_METERS / 2, DriveTrainConstants.WHEEL_BASE_METERS / 2),
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
        this.autonomousChooser = new LoggedDashboardChooser<Command>("Auto Routine", AutoBuilder.buildAutoChooser());
        this.autonomousChooser.addOption("Station 3 - Shoot and Move Away", getShootAndMoveAway());
        this.autonomousChooser.addOption("Station 1 - Drive First", getBlue1DriveFirst());
        this.autonomousChooser.addOption("2056", get2056());

        // Add chooser to the shuffleboard
        // ShuffleboardTab tab = Shuffleboard.getTab("Driver");
        // tab.add("Autonomous", this.autonomousChooser).withSize(2, 1).withPosition(0, 0);
    }

    /**
     * 
     */
    public LoggedDashboardChooser<Command> getAutonomousChooser() {
        return this.autonomousChooser;
    }

    /**
     * 
     */
    private Command getShootAndMoveAway() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Station3 - Center5");

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
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.PAUSE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
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
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Station 1 - Drive First");

        if (pathGroup == null) {
            return Commands.print("*** Failed to build Blue1DriveFirst");
        }

        Command command = Commands.sequence(
            Commands.print("*** Starting Blue1DriveFirst ***"),

            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.PAUSE);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.PAUSE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(0))
            ),
            new WaitUntilCommand(this.shooterSubsystem::hasNote),
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
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            new WaitUntilCommand(this.shooterSubsystem::hasNote),
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
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(4))
            ),

            Commands.print("*** Finished Blue1DriveFirst ***")
        );

        return command;
    }

    /**
     * 
     */
    private Command get2056() {
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("2056");

        if (pathGroup == null) {
            return Commands.print("*** Failed to build 2056");
        }

        // Set the note if running in simulator
        if (Robot.isSimulation()) {
            this.shooterSubsystem.setHasNote(true);
        }

        Command command = Commands.sequence(
            Commands.print("*** Starting 2056 ***"),
            
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.SPINUP_FOR_SPEAKER);
                }),
                AutoBuilder.followPath(pathGroup.get(0))
            ),
            new WaitUntilCommand(this.armSubsystem::isAtAngle),
            new WaitUntilCommand(this.shooterSubsystem::isSpunupForSpeaker),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(1))
            ),
            new WaitUntilCommand(this.shooterSubsystem::hasNote),
            new ParallelCommandGroup(
                AutoBuilder.followPath(pathGroup.get(2)),
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.SPINUP_FOR_SPEAKER);
                })
            ),
            new WaitUntilCommand(this.armSubsystem::isAtAngle),
            new WaitUntilCommand(this.shooterSubsystem::isSpunupForSpeaker),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new ParallelCommandGroup(
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
                    this.intakeSubsystem.addAction(IntakeSubsystem.Action.INTAKE);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.INTAKE);
                }),
                AutoBuilder.followPath(pathGroup.get(3))
            ),
            new WaitUntilCommand(this.shooterSubsystem::hasNote),
            new ParallelCommandGroup(
                AutoBuilder.followPath(pathGroup.get(4)),
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.SPINUP_FOR_SPEAKER);
                })
            ),
            new WaitUntilCommand(this.armSubsystem::isAtAngle),
            new WaitUntilCommand(this.shooterSubsystem::isSpunupForSpeaker),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new ParallelDeadlineGroup(
                new TargetAndGrabNoteCommand(
                    this.armSubsystem,
                    this.intakeSubsystem,
                    this.shooterSubsystem,
                    this.visionSubsystem::getBestNoteTarget),
                AutoBuilder.followPath(pathGroup.get(5))
            ),
            new ParallelCommandGroup(
                AutoBuilder.pathfindToPoseFlipped(
                    FieldConstants.AUTONOMOUS_SHOOTING_POSE,
                    constraints, 0.0, 0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                ),
                new InstantCommand(() -> {
                    this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_SPEAKER);
                    this.shooterSubsystem.addAction(ShooterSubsystem.Action.SPINUP_FOR_SPEAKER);
                })
            ),
            new InstantCommand(() -> {
                this.shooterSubsystem.addAction(ShooterSubsystem.Action.SHOOT);
            }),
            new WaitUntilCommand(this.shooterSubsystem::hasShot),
            new InstantCommand(() -> {
                this.armSubsystem.addAction(ArmSubsystem.Action.MOVE_TO_INTAKE);
            }),

            Commands.print("*** Finished 2056 ***")
        );

        return command;
    }
}
