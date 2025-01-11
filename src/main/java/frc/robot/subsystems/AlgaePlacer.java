package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePlacer extends SubsystemBase {
    private final TalonFX topMotor;

    public AlgaePlacer() {
        topMotor = new TalonFX(32);
    }

    public Command SpinSpeed(DoubleSupplier speed) {
        return run(() -> {
            topMotor.set(speed.getAsDouble());
        }).finallyDo(() -> {topMotor.set(0);})
        .withName("Spin speed");
    }
}
