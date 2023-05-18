package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AutoLevelCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.FeedForwardCharacterizationCommand;
import frc.robot.commands.FeedForwardCharacterizationCommand.FeedForwardCharacterizationData;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Action;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class AutonomousBuilder {
    private final PathConstraints CONSTRAINTS = new PathConstraints(4, 3);

    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final ArmSubsystem armSubsystem;

    private final SendableChooser<Command> autoChooser;
    private final HashMap<String, Command> eventMap;
    
    private HashMap<String, List<PathPlannerTrajectory>> trajectoryMap;

    /**
     * 
     */
    public AutonomousBuilder(SwerveDriveSubsystem swerveDrive, PoseEstimatorSubsystem poseEstimator, ArmSubsystem armSubsystem) {
        this.swerveDrive = swerveDrive;
        this.poseEstimator = poseEstimator;
        this.armSubsystem = armSubsystem;
        this.eventMap = getEventMap();
        
        setTrajectoryMap();
        
        this.autoChooser = new SendableChooser<>();
        this.autoChooser.setDefaultOption("None", Commands.none());

        if (RobotConstants.TUNING_MODE) {
            this.autoChooser.addOption("PPDrive5Meters", getPPCommand("PPDrive5Meters"));
            this.autoChooser.addOption("PPTurn180Degrees", getPPCommand("PPTurn180Degrees"));

            this.autoChooser.addOption("DPDrive5Meters", new DPDrive5Meters(this.swerveDrive, this.poseEstimator));
            this.autoChooser.addOption("DPTurn180Degrees", new DPTurn180Degrees(this.swerveDrive, this.poseEstimator));

            this.autoChooser.addOption("Drive Velocity Tuning",
                Commands.sequence(
                    Commands.runOnce(this.swerveDrive::disableFieldRelative, this.swerveDrive),
                    Commands.deadline(
                        Commands.waitSeconds(5.0),
                        Commands.run(() -> this.swerveDrive.drive(1.5, 0.0, 0.0, new Rotation2d(), false), this.swerveDrive)
                    )
                )
            );

            this.autoChooser.addOption("Drive Characterization",
                new FeedForwardCharacterizationCommand(
                    this.swerveDrive,
                    true,
                    new FeedForwardCharacterizationData("drive"),
                    this.swerveDrive::runCharacterizationVolts,
                    this.swerveDrive::getCharacterizationVelocity)
                );
        }

        this.autoChooser.addOption("PPTwoPieceBalance", getPPTwoPieceBalance());
        this.autoChooser.addOption("DPTwoPieceBalance", new DPTwoPieceBalance(this.swerveDrive, this.poseEstimator, this.armSubsystem));
    }

    /**
     *
     */
    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Autonomous", this.autoChooser).withSize(2, 1).withPosition(0, 0);
    }

    /**
     * Gets the currently selected auto command
     * @return auto command
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
    
    /**
     * 
     */
    private HashMap<String, Command> getEventMap() {
        HashMap<String, Command> eventMap = new HashMap<String, Command>();

        eventMap.put("CloseGripper", Commands.print("Grabbing Cube").withTimeout(3.0));
        eventMap.put("OpenGripper", Commands.print("Placing Cube").withTimeout(3.0));
        
        return eventMap;
    }
    
    /**
     * 
     */
    private void setTrajectoryMap() {
        this.trajectoryMap = new HashMap<String,List<PathPlannerTrajectory>>();

        try {
            Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner"));

            files.filter(file -> !Files.isDirectory(file))
                .map(Path::getFileName)
                .map(Path::toString)
                .filter(fileName -> fileName.endsWith(".path"))
                .sorted()
                .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
                .forEach(pathName -> {
                    List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(pathName, CONSTRAINTS);

                    if (trajectories != null) {
                        this.trajectoryMap.put(pathName, trajectories);
                    }
                });

            files.close();

        } catch (IOException e) {
            System.out.println("*** Failed to load paths ***");
        }
    }

    /**
     * 
     */
    private Command getPPTwoPieceBalance() {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("PPTwoPieceBalance", CONSTRAINTS);

        if (path == null) {
            return Commands.print("*** Failed to build TwoPieceBalance");
        }

        Logger.getInstance().recordOutput("Path", path.get(0));
    
        Command command = Commands.sequence(
            Commands.print("*** Starting PPTwoPieceBalance ***"),
            new InstantCommand(() -> {
                    this.armSubsystem.addAction(Action.MOVE_TO_DRAWER);
                    this.armSubsystem.addAction(Action.GRAB);
                    this.armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
                    this.armSubsystem.addAction(Action.RELEASE);
                    this.armSubsystem.addAction(Action.PAUSE);
                    this.armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            new FollowPathWithEvents(new DriveTrajectoryCommand(this.swerveDrive, this.poseEstimator, path.get(0), true), path.get(0).getMarkers(), this.eventMap),
            new InstantCommand(() -> {
                this.armSubsystem.addAction(Action.GRAB);
                this.armSubsystem.addAction(Action.MOVE_TO_HIGH_NODE);
            }),
            new FollowPathWithEvents(new DriveTrajectoryCommand(this.swerveDrive, this.poseEstimator, path.get(1), false), path.get(1).getMarkers(), this.eventMap),
            new InstantCommand(() -> {
                this.armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
                this.armSubsystem.addAction(Action.RELEASE);
                this.armSubsystem.addAction(Action.PAUSE);
                this.armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            new FollowPathWithEvents(new DriveTrajectoryCommand(this.swerveDrive, this.poseEstimator, path.get(2), false), path.get(2).getMarkers(), this.eventMap),
            new InstantCommand(() -> {
                this.armSubsystem.addAction(Action.GRAB);
                this.armSubsystem.addAction(Action.MOVE_TO_DRAWER);
            }),
            new FollowPathWithEvents(new DriveTrajectoryCommand(this.swerveDrive, this.poseEstimator, path.get(3), false), path.get(3).getMarkers(), this.eventMap),
            new AutoLevelCommand(this.swerveDrive),
            Commands.print("*** Finished PPTwoPieceBalance ***")
        );

        return command;
    }

    /**
     * 
     */
    private Command getPPCommand(String name) {
        List<PathPlannerTrajectory> path = this.trajectoryMap.get(name);

        if (path == null) {
            return Commands.print("*** Failed to build " + name);
        }
        
        Command command = Commands.sequence(
            Commands.print("*** Starting " + name + " ***"),
            new FollowPathWithEvents(new DriveTrajectoryCommand(this.swerveDrive, this.poseEstimator, path.get(0), true), path.get(0).getMarkers(), this.eventMap),
            Commands.print("*** Finished " + name + " ***")
        );

        return command;
    }
}