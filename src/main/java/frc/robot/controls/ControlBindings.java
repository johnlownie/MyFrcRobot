package frc.robot.controls;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBindings {
    Optional<Trigger> driveToCube();
    Optional<Trigger> driveToPoleLeft();
    Optional<Trigger> driveToPoleRight();
    Optional<Trigger> driveTeleop();
    Optional<Trigger> driveType();
    Optional<Trigger> resetPose();
    Optional<Trigger> reseedSteerMotors();
    Optional<Trigger> xStance();
    DoubleSupplier translationX();
    DoubleSupplier translationY();
    DoubleSupplier omega();
    Supplier<Optional<Rotation2d>> heading();
    BooleanSupplier driverWantsControl();
}
