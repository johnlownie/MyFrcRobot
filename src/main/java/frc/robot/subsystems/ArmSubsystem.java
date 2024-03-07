package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;
import frc.lib.util.Timer;
import frc.robot.Constants.ArmConstants;
import frc.robot.modules.arm.ArmModule;

/**
 * 
 */
public class ArmSubsystem extends SubsystemBase {
    private final ArmModule armModule;

    public static enum Action {
        CLIMB, IDLE, MOVE_TO_AMP, MOVE_TO_INTAKE, MOVE_TO_SPEAKER, MOVE_TO_STAGE, PAUSE, STOP
    }

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    private boolean is_at_angle;
    private boolean notify_at_angle;
    private Timer timer;

    /**
     * 
     */
    public ArmSubsystem(ArmModule armModule) {
        this.armModule = armModule;

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>("ARM SUBSYSTEM");
        this.stateMachine.setDefaultState(Action.MOVE_TO_INTAKE, this::handleMoveToIntake);
        this.stateMachine.addState(Action.CLIMB, this::handleClimb);
        this.stateMachine.addState(Action.IDLE, this::handleIdle);
        this.stateMachine.addState(Action.MOVE_TO_AMP, this::handleMoveToAmp);
        this.stateMachine.addState(Action.MOVE_TO_SPEAKER, this::handleMoveToSpeaker);
        this.stateMachine.addState(Action.MOVE_TO_STAGE, this::handleMoveToStage);
        this.stateMachine.addState(Action.PAUSE, this::handlePause);
        this.stateMachine.addState(Action.STOP, this::handleStop);

        this.actionQueue = new LinkedList<Action>();

        this.is_at_angle = false;
        this.notify_at_angle = false;
        this.timer = new Timer();
    }

    /**
     * 
     */
    public void addAction(Action action) {
        this.actionQueue.add(action);
    }

    /**
     * 
     */
    private void handleClimb(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            this.armModule.climb();
        }

        if (this.timer.hasElapsed(2)) {
            this.timer.stop();
            this.stateMachine.setState(Action.STOP);
        }
    }
    

    /**
     * 
     */
    private void handleIdle(StateMetadata<Action> stateMetadata) {
    }
    
    /**
     * 
     */
    private void handleMoveToAmp(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            setDesiredAngle(ArmConstants.ANGLE_AMP);
            this.is_at_angle = false;
            this.notify_at_angle = true;
        }
    }

    /**
     * 
     */
    private void handleMoveToIntake(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            setDesiredAngle(ArmConstants.ANGLE_INTAKE);
            this.is_at_angle = false;
            this.notify_at_angle = true;
        }
    }

    /**
     * 
     */
    private void handleMoveToSpeaker(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            setDesiredAngle(ArmConstants.ANGLE_SPEAKER);
            this.is_at_angle = false;
            this.notify_at_angle = true;
        }
    }

    /**
     * 
     */
    private void handleMoveToStage(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            setDesiredAngle(ArmConstants.ANGLE_STAGE);
            this.is_at_angle = false;
            this.notify_at_angle = true;
        }
    }

    /**
     * 
     */
    private void handlePause(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
        }

        if (this.timer.hasElapsed(1)) {
            this.timer.stop();
            this.stateMachine.setState(Action.IDLE);
        }
    }

    /**
     * 
     */
    private void handleStop(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.armModule.stop();
        }
    }
    
    /**
     * 
     */
    private boolean isActionComplete() {
        return (this.armModule.isAtAngle() || this.stateMachine.getCurrentState() == Action.IDLE) && !this.timer.isRunning();
    }

    /**
     * 
     */
    public boolean isAtAngle() {
        return this.is_at_angle;
    }
    
    @Override
    public void periodic() {
        this.stateMachine.update();
        this.armModule.update();

        // actions run for no longer than 3 seconds
        if (this.timer.isRunning() && this.timer.hasElapsed(3)) {
            this.timer.stop();
        }

        if (isActionComplete()) {
            if (this.notify_at_angle) {
                this.is_at_angle = true;
                this.notify_at_angle = false;
            }

            this.stateMachine.setState(Action.IDLE);
        }

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.IDLE && this.actionQueue.size() > 0) {
            try { 
                Action nextAction = this.actionQueue.removeFirst();
                this.stateMachine.setState(nextAction);
            } catch (NoSuchElementException e) {}
        }

        Logger.recordOutput("Subsystems/Arm/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/Arm/IsAtAngle", isAtAngle());
    }

    /**
     * 
     */
    public void updatePID(double kP, double kI, double kD) {
        this.armModule.updatePID(kP, kI, kD);
    }

    /**
     * Getters and Setters
     */
    public void setDesiredAngle(double desired_angle_degrees) { this.armModule.setDesiredAngle(desired_angle_degrees); }
}