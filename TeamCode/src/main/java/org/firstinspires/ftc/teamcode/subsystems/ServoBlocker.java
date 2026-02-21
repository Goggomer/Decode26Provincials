package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
public class ServoBlocker implements Subsystem {
    public static final ServoBlocker INSTANCE = new ServoBlocker();
    private ServoBlocker() { }

    private ServoEx servo = new ServoEx("blocker");

    public Command up = new SetPosition(servo, 0.6).requires(this);
    public Command down = new SetPosition(servo, 0.05).requires(this);
}