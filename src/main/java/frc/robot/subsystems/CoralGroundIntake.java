package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGroundIntake extends SubsystemBase {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    public CoralGroundIntake() {
        topMotor = new TalonFX(32);
        bottomMotor = new TalonFX(33);
    }

    public Command SpinSpeed(DoubleSupplier speed) {
        return run(() -> {
            topMotor.set(speed.getAsDouble());
            bottomMotor.set(speed.getAsDouble());
        }).finallyDo(() -> {topMotor.set(0); bottomMotor.set(0);})
        .withName("Spin speed");
    }
}
