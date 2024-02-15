package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMetadata;
import frc.lib.util.Timer;
import frc.robot.modules.shooter.ShooterModule;

/**
 * 
 */
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterModule shooterModule;

    public static enum Action { 
        IDLE, SHOOT_AMP, SHOOT_SPEAKER
    }

    private final StateMachine<Action> stateMachine;
    private final LinkedList<Action> actionQueue;

    private Timer timer;
    private boolean has_shot;
    private boolean notify_on_shoot;

    /**
     * 
     */
    public ShooterSubsystem(ShooterModule shooterModule) {
        this.shooterModule = shooterModule;

        // Sets states for the arm, and what methods.
        this.stateMachine = new StateMachine<>("SHOOTER SUBSYSTEM");
        this.stateMachine.setDefaultState(Action.IDLE, this::handleIdle);
        this.stateMachine.addState(Action.SHOOT_AMP, this::handleShootAmp);
        this.stateMachine.addState(Action.SHOOT_SPEAKER, this::handleShootSpeaker);

        this.actionQueue = new LinkedList<Action>();

        this.timer = new Timer();
        this.has_shot = false;
        this.notify_on_shoot = false;
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
            this.shooterModule.stop();
        }
    }

    /**
     * 
     */
    private void handleShootAmp(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            this.has_shot = false;
            this.notify_on_shoot = true;

            this.shooterModule.spinupForAmp();
        }

        if (this.shooterModule.isSpunupForAmp()) {
            this.shooterModule.shoot();
        }
    }

    /**
     * 
     */
    private void handleShootSpeaker(StateMetadata<Action> stateMetadata) {
        if (stateMetadata.isFirstRun()) {
            this.timer.reset();
            this.timer.start();
            this.has_shot = false;
            this.notify_on_shoot = true;
            
            this.shooterModule.spinupForSpeaker();
        }

        if (this.shooterModule.isSpunupForSpeaker()) {
            this.shooterModule.shoot();
        }
    }
    
    /**
     * 
     */
    public boolean hasShot() {
        boolean has_shot = this.has_shot;
        this.has_shot = false;

        return has_shot;
    }
    
    /**
     * 
     */
    private boolean isActionComplete() {
        return !this.timer.isRunning();
    }
    
    @Override
    public void periodic() {
        this.stateMachine.update();
        this.shooterModule.update();

        // actions run for no longer than 2 seconds
        if (this.timer.hasElapsed(2)) {
            this.timer.stop();
        }

        if (isActionComplete()) {
            if (this.notify_on_shoot) {
                this.has_shot = true;
                this.notify_on_shoot = false;
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

        Logger.recordOutput("Subsystems/Shooter/Current State", this.stateMachine.getCurrentState());
    }
}
