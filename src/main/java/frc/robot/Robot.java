// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.containers.ChargedUpContainer;
import frc.robot.containers.RobotContainer;
import frc.robot.containers.SimulatorContainer;
import frc.robot.utils.Alert;
import frc.robot.utils.BatteryTracker;
import frc.robot.utils.Alert.AlertType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Alliance alliance = Alliance.Invalid;
    private int location = 0;
    private Command autonomousCommand;

    private final Alert logReceiverQueueAlert = new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);

    /**
     * 
     */
    public Robot() {
        super(RobotConstants.LOOP_PERIOD_SECS);
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // from AdvantageKit Robot Configuration docs
        // (https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/START-LOGGING.md#robot-configuration)

        Logger logger = Logger.getInstance();

        // Log build stats
        logger.recordMetadata("Robot", RobotConstants.getRobot().toString());
        logger.recordMetadata("BatteryName", BatteryTracker.scanBattery(1.0));
        logger.recordMetadata("TuningMode", Boolean.toString(RobotConstants.TUNING_MODE));
        logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        String gitDirty = "Unknown";
        switch (BuildConstants.DIRTY) {
            case 0:
                gitDirty = "All changes committed";
                break;
            case 1:
                gitDirty = "Uncomitted changes";
                break;
        }
        logger.recordMetadata("GitDirty", gitDirty);

        // Set robotContainer
        switch (RobotConstants.getMode()) {
            case REAL:
                // Provide log data over the network, viewable in Advantage Scope.
                logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
                logger.addDataReceiver(new NT4Publisher());
                LoggedPowerDistribution.getInstance();

                this.robotContainer = new ChargedUpContainer();
                break;
      
            case SIM:
                logger.addDataReceiver(new WPILOGWriter(""));
                logger.addDataReceiver(new NT4Publisher());

                this.robotContainer = new SimulatorContainer();
                break;
      
            case REPLAY:
                // Run as fast as possible during replay
                // Prompt the user for a file path on the command line (if not open in AdvantageScope)
                // Read log file for replay
                // Save replay results to a new log with the "_sim" suffix
                setUseTiming(false);
                String path = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(path));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));

                this.robotContainer = new SimulatorContainer();
                break;
          }

        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        logger.start();

        // Alternative logging of scheduled commands
        CommandScheduler.getInstance().onCommandInitialize(command -> Logger.getInstance().recordOutput("Command initialized", command.getName()));
        CommandScheduler.getInstance().onCommandInterrupt(command -> Logger.getInstance().recordOutput("Command interrupted", command.getName()));
        CommandScheduler.getInstance().onCommandFinish(command -> Logger.getInstance().recordOutput("Command finished", command.getName()));

        checkDriverStationUpdate();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        checkDriverStationUpdate();

        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

        Logger.getInstance().recordOutput("Mechanisms/Visualization", MechanismConstants.CANVAS);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        checkDriverStationUpdate();

        this.autonomousCommand = this.robotContainer.getAutonomousCommand();

        // schedule the autonomous command
        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }

        checkDriverStationUpdate();
        this.robotContainer.enable();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // this.robotContainer.disable();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        // this.robotContainer.disabledPeriodic();
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }

    /**
     * Checks the driverstation alliance. We have have to check repeatedly because
     * we don't know when the
     * driverstation/FMS will connect, and the alliance can change at any time in
     * the shop.
     */
    private void checkDriverStationUpdate() {
        // https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
        Alliance currentAlliance = DriverStation.getAlliance();
        int currentLocation = DriverStation.getLocation();

        // If we have data, and have a new alliance from last time
        if (DriverStation.isDSAttached() && (currentAlliance.compareTo(this.alliance) != 0 || currentLocation != this.location)) {
            this.robotContainer.onAllianceChanged(currentAlliance, currentLocation);
            this.alliance = currentAlliance;
            this.location = currentLocation;
        }
    }
}
