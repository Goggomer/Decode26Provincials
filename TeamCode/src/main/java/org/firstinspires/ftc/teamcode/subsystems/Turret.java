package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private final MotorEx motor = new MotorEx("Turret")
            .brakeMode();

    private final ControlSystem controller = ControlSystem.builder()
            .posPid(0.02, 0, 0.001)
            .build();

    public final Command tclose = new SequentialGroup(
            new RunToPosition(controller, 270).requires(this),
            new InstantCommand(motor::zeroed)
    );

    public final Command tfar = new SequentialGroup(
            new RunToPosition(controller, 290).requires(this),
            new InstantCommand(motor::zeroed)
    );

    public final Command tcloser = new SequentialGroup(
            new RunToPosition(controller, 180).requires(this),
            new InstantCommand(motor::zeroed)
    );

    public final Command tfarr = new SequentialGroup(
            new RunToPosition(controller, 190).requires(this),
            new InstantCommand(motor::zeroed)
    );

    @Override
    public void periodic() {
        motor.setPower(controller.calculate(motor.getState()));
    }
}