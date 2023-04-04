package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoLevelCommand;
import frc.robot.commands.DrivePathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.Action;

public class AutonomousBuilder {
    private final PathConstraints CONSTRAINTS = new PathConstraints(4, 3);

    private final SwerveDriveSubsystem swerveDrive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final ArmSubsystem armSubsystem;

    private final SendableChooser<Command> autoChooser;
    private final SwerveAutoBuilder swerveAutoBuilder;
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

        this.swerveAutoBuilder = new AutoBuilder(
            this.poseEstimator::getCurrentPose, this.poseEstimator::setCurrentPose, this.swerveDrive.getKinematics(),
            new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0), this.swerveDrive::setModuleStates,
            this.eventMap, true, new Command[] {}, this.swerveDrive
        );
        
        setTrajectoryMap();
        
        this.autoChooser = new SendableChooser<>();
        this.autoChooser.setDefaultOption("None", Commands.none());
        this.autoChooser.addOption("PPDrive5Meters", getPPCommand("PPDrive5Meters"));
        this.autoChooser.addOption("PPTurn180Degrees", getPPCommand("PPTurn180Degrees"));
        this.autoChooser.addOption("PPTwoPieceBalance", getTwoPieceBalance());
        this.autoChooser.addOption("DPDrive5Meters", new Drive5Meters(this.swerveDrive, this.poseEstimator));
        this.autoChooser.addOption("DPTurn180Degrees", new Turn180Degrees(this.swerveDrive, this.poseEstimator));
        this.autoChooser.addOption("DPTwoPieceBalance", new TwoPieceBalance(this.swerveDrive, this.poseEstimator, this.armSubsystem, getAutoBuildForPathGroup("PoletoPiece")));
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
    private Command getAutoBuildForPathGroup(String pathGroupName) {
        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(pathGroupName, new PathConstraints(4, 3));
        if (trajectories == null) {
            return Commands.print("*** Path failed to load: " + pathGroupName);
        }

        return this.swerveAutoBuilder.fullAuto(trajectories);
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
    private Command getTwoPieceBalance() {
        List<PathPlannerTrajectory> path = this.trajectoryMap.get("PPTwoPieceBalance");

        if (path == null) {
            return Commands.print("*** Failed to build TwoPieceBalance");
        }
        
        Command command = Commands.sequence(
            Commands.print("*** Starting PPTwoPieceBalance ***"),
            new InstantCommand(() -> {
                    armSubsystem.addAction(Action.MOVE_TO_DRAWER);
                    armSubsystem.addAction(Action.GRAB);
                    armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
                    armSubsystem.addAction(Action.RELEASE);
                    armSubsystem.addAction(Action.PAUSE);
                    armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            new FollowPathWithEvents(new DrivePathCommand(this.swerveDrive, this.poseEstimator, path.get(0), true), path.get(0).getMarkers(), this.eventMap),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.GRAB);
                armSubsystem.addAction(Action.MOVE_TO_MID_NODE);
            }),
            new FollowPathWithEvents(new DrivePathCommand(this.swerveDrive, this.poseEstimator, path.get(1), false), path.get(1).getMarkers(), this.eventMap),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.RELEASE);
                armSubsystem.addAction(Action.PAUSE);
                armSubsystem.addAction(Action.MOVE_TO_GROUND);
            }),
            new WaitUntilCommand(armSubsystem::isReleased),
            new FollowPathWithEvents(new DrivePathCommand(this.swerveDrive, this.poseEstimator, path.get(2), false), path.get(2).getMarkers(), this.eventMap),
            new InstantCommand(() -> {
                armSubsystem.addAction(Action.GRAB);
                armSubsystem.addAction(Action.MOVE_TO_DRAWER);
            }),
            new FollowPathWithEvents(new DrivePathCommand(this.swerveDrive, this.poseEstimator, path.get(3), false), path.get(2).getMarkers(), this.eventMap),
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
            new FollowPathWithEvents(new DrivePathCommand(this.swerveDrive, this.poseEstimator, path.get(0), true), path.get(0).getMarkers(), this.eventMap),
            Commands.print("*** Finished " + name + " ***")
        );

        return command;
    }
}