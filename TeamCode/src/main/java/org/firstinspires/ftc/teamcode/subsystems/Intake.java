package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private final MotorEx intake = new MotorEx("intake");

    public final Command off = new SetPower(intake, 0).requires(this);
    public final Command on = new SetPower(intake, 1).requires(this);
    public final Command reverse = new SetPower(intake, -1).requires(this);
}