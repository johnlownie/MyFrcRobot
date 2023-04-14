package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.StateMachine;
import frc.lib.util.StateMetadata;
import frc.lib.util.Timer;
import frc.robot.Constants.ArmConstants;
import frc.robot.modules.arm.ArmModule;

/**
 * 
 */
public class ArmSubsystem extends SubsystemBase {
    public static enum Action {
        IDLE, GRAB, MOVE, MOVE_TO_DRAWER, MOVE_TO_GROUND, MOVE_TO_LOW_NODE, MOVE_TO_MID_NODE, MOVE_TO_HIGH_NODE, PAUSE, RELEASE, STOP
    }

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    private final ArmModule armModule;

    private boolean is_released;
    private boolean notify_on_release;
    private Timer timer;

    /**
     * 
     */
    public ArmSubsystem(ArmModule armModule) {
        this.armModule = armModule;

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>("ARM SUBSYSTEM");
        this.stateMachine.setDefaultState(Action.STOP, this::handleStop);
        this.stateMachine.addState(Action.GRAB, this::handleGrab);
        this.stateMachine.addState(Action.IDLE, this::handleIdle);
        this.stateMachine.addState(Action.MOVE, this::handleMove);
        this.stateMachine.addState(Action.MOVE_TO_DRAWER, this::handleMoveToDrawer);
        this.stateMachine.addState(Action.MOVE_TO_GROUND, this::handleMoveToGround);
        this.stateMachine.addState(Action.MOVE_TO_LOW_NODE, this::handleMoveToLowNode);
        this.stateMachine.addState(Action.MOVE_TO_MID_NODE, this::handleMoveToMidNode);
        this.stateMachine.addState(Action.MOVE_TO_HIGH_NODE, this::handleMoveToHighNode);
        this.stateMachine.addState(Action.PAUSE, this::handlePause);
        this.stateMachine.addState(Action.RELEASE, this::handleRelease);

        this.actionQueue = new LinkedList<Action>();

        this.is_released = false;
        this.notify_on_release = false;
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
    public void closeGripper() {
        this.actionQueue.add(Action.GRAB);
    }

    public void disable() {
        this.armModule.stop();
        this.actionQueue.clear();
    }
    
    /**
     * 
     */
    private void handleGrab(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.armModule.close();
        }

        this.stateMachine.setState(Action.IDLE);
    }

    /**
     * 
     */
    private void handleIdle(StateMetadata<Action> stateMetadata) {
    }

    /**
     * 
     */
    private void handleMove(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.is_released = false;
            this.armModule.setEnabled(true);
        }
    }
    
    /**
     * 
     */
    private void handleMoveToDrawer(StateMetadata<Action> stateMetadata) {
        setDesiredAngle(ArmConstants.ANGLE_DRAWER_PICKUP);
        moveArm(false);
    }
    
    /**
     * 
     */
    private void handleMoveToGround(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            setDesiredAngle(ArmConstants.ANGLE_GROUND_PICKUP);
            openGripper();
            moveArm(false);
        }
    }

    /**
     * 
     */
    private void handleMoveToLowNode(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            setDesiredAngle(ArmConstants.ANGLE_DEPLOY_LOW);
            moveArm(true);
        }
    }

    /**
     * 
     */
    private void handleMoveToMidNode(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            setDesiredAngle(ArmConstants.ANGLE_DEPLOY_MID);
            moveArm(true);
        }
    }

    /**
     * 
     */
    private void handleMoveToHighNode(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            setDesiredAngle(ArmConstants.ANGLE_DEPLOY_HIGH);
            moveArm(false);
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
            timer.stop();
            this.stateMachine.setState(Action.IDLE);
        }
    }
    
    /**
     * 
     */
    private void handleRelease(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.armModule.open();
        }

        this.stateMachine.setState(Action.IDLE);
    }

    /**
     * 
     */
    private void handleStop(StateMetadata<Action> stateMetadata) {
        this.armModule.stop();
    }

    /**
     * 
     */
    private boolean isActionComplete() {
        return (this.armModule.atAngle() || this.stateMachine.getCurrentState() == Action.IDLE) && !this.timer.isRunning();
    }

    /**
     * 
     */
    public boolean isReleased() {
        return this.is_released;
    }

    /**
     * 
     */
    private void moveArm(boolean notify_on_release) {
        this.notify_on_release = notify_on_release;
        this.stateMachine.setState(Action.MOVE);
    }
    
    /**
     * 
     */
    public void openGripper() {
        this.stateMachine.setState(Action.RELEASE);
    }

    @Override
    public void periodic() {
        this.stateMachine.update();
        this.armModule.update();

        if (isActionComplete()) {
            if (this.notify_on_release) {
                this.is_released = true;
                this.notify_on_release = false;
            }

            this.stateMachine.setState(Action.IDLE);
        }

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.IDLE) {
            try { 
                
                Action nextAction = this.actionQueue.removeFirst();

                this.stateMachine.setState(nextAction);
            
            } catch (NoSuchElementException e) {}
        }
    }

    /**
     * Getters and Setters
     */
    public void setDesiredAngle(double desired_angle_degrees) { this.armModule.setDesiredAngle(desired_angle_degrees); }
}