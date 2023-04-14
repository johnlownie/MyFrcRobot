package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.StateMachine;
import frc.lib.util.StateMetadata;
import frc.robot.modules.drawer.DrawerModule;

/**
 * 
 */
public class DrawerSubsystem extends SubsystemBase {
    public static enum Action {
        EXTEND, EXTEND_AND_WAIT, HOLD, RETRACT
    }
    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    private final DrawerModule drawerModule;

    /**
     * 
     */
    public DrawerSubsystem(DrawerModule drawerModule) {
        this.drawerModule = drawerModule;

        // Sets states for the drawer, and what methods.
        this.stateMachine = new StateMachine<>("DRAWER SUBSYSTEM");
        this.stateMachine.setDefaultState(Action.HOLD, this::handleHold);
        this.stateMachine.addState(Action.EXTEND, this::handleExtend);
        this.stateMachine.addState(Action.EXTEND_AND_WAIT, this::handleExtendAndWait);
        this.stateMachine.addState(Action.RETRACT, this::handleRetract);

        this.actionQueue = new LinkedList<Action>();
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
    public void extend() {
        this.actionQueue.add(Action.EXTEND);
    }

    /**
     * 
     */
    public void extendAndWait() {
        this.actionQueue.add(Action.EXTEND_AND_WAIT);
    }

    /**
     * 
     */
    private void handleExtend(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.drawerModule.extend();
        }

        this.stateMachine.setState(Action.HOLD);
    }

    /**
     * 
     */
    private void handleExtendAndWait(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.drawerModule.extend();
        }

        if (this.drawerModule.hasGamePiece()) {
            this.stateMachine.setState(Action.RETRACT);
        }
    }

    /**
     * 
     */
    private void handleHold(StateMetadata<Action> stateMetadata) {
    }

    /**
     * 
     */
    private void handleRetract(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.drawerModule.retract();
        }

        this.stateMachine.setState(Action.HOLD);
    }

    /**
     * 
     */
    public boolean hasGamePiece() {
        return this.drawerModule.hasGamePiece();
    }

    @Override
    public void periodic() {
        this.stateMachine.update();
        this.drawerModule.update();

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.HOLD) {
            try { 
                
                Action nextAction = this.actionQueue.removeFirst();

                this.stateMachine.setState(nextAction);
            
            } catch (NoSuchElementException e) {}
        }
    }

    /**
     * 
     */
    public void retract() {
        this.actionQueue.add(Action.RETRACT);
    }
}
