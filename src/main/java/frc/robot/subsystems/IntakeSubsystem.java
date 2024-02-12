package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.StateMachine;
import frc.lib.util.StateMetadata;
import frc.lib.util.Timer;
import frc.robot.modules.intake.IntakeModule;

/**
 * 
 */
public class IntakeSubsystem extends SubsystemBase {
    private final IntakeModule intakeModule;

    public static enum Action {
        IDLE, EJECT, INTAKE
    }

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    private Timer timer;
    
    /**
     * 
     */
    public IntakeSubsystem(IntakeModule intakeModule) {
        this.intakeModule = intakeModule;

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>("INTAKE SUBSYSTEM");
        this.stateMachine.setDefaultState(Action.IDLE, this::handleIdle);
        this.stateMachine.addState(Action.EJECT, this::handleEject);
        this.stateMachine.addState(Action.INTAKE, this::handleIntake);

        this.actionQueue = new LinkedList<Action>();

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
    private void handleIdle(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.intakeModule.stop();
        }
    }

    /**
     * 
     */
    private void handleEject(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            this.intakeModule.eject();
        }
    }

    /**
     * 
     */
    private void handleIntake(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            this.intakeModule.intake();
        }
    }
    
    /**
     * 
     */
    private boolean isActionComplete() {
        return this.intakeModule.isIntakeLimitSwitchTriggered() || !this.timer.isRunning();
    }
    
    @Override
    public void periodic() {
        this.stateMachine.update();
        this.intakeModule.update();

        // actions run for no longer than 2 seconds
        if (this.timer.hasElapsed(2)) {
            this.timer.stop();
        }

        if (isActionComplete()) {
            this.stateMachine.setState(Action.IDLE);
        }

        // Run any action in the queue
        if (this.stateMachine.getCurrentState() == Action.IDLE && this.actionQueue.size() > 0) {
            try { 
                Action nextAction = this.actionQueue.removeFirst();
                this.stateMachine.setState(nextAction);
            } catch (NoSuchElementException e) {}
        }
    }
}
