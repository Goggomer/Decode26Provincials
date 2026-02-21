package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();

    private Robot() {
        super(
                Intake.INSTANCE,
                FlyNew.INSTANCE,
                ServoBlocker.INSTANCE,
                Transfer.INSTANCE,
                Turret.INSTANCE
        );
    }
    public final Command RobotOff =
            new ParallelGroup(
                    Intake.INSTANCE.off,
                    FlyNew.INSTANCE.off,
                    Transfer.INSTANCE.off
            );
    public final Command IntakeOn =
            new SequentialGroup(
                    ServoBlocker.INSTANCE.down,
                    Intake.INSTANCE.on,
                    Transfer.INSTANCE.slow
            );
    public final Command ShootClose =
            new SequentialGroup(
                    // 1. Tell the flywheel to start (This finishes instantly)
                    FlyNew.INSTANCE.slow,

                    // 2. Prepare the hardware
                    ServoBlocker.INSTANCE.down,
                    Transfer.INSTANCE.off,
                    Intake.INSTANCE.off,

                    // 3. THE WAIT: The sequence stops here until the motor is actually fast enough
                    new WaitUntil(FlyNew.INSTANCE::isAtSpeed),

                    // 4. FIRE: This only runs AFTER isAtSpeed returns true
                    ServoBlocker.INSTANCE.up,
                    new Delay(1),
                    Transfer.INSTANCE.on,

                    // 5. SUCCESS: Keep things running for a moment to let the ball exit
                    new Delay(0.5),

                    // Optional: Turn intake back on or flywheel off
                    Intake.INSTANCE.on
            );

    public final Command ShootFar =
            new SequentialGroup(
                    // 1. Tell the flywheel to start (This finishes instantly)
                    FlyNew.INSTANCE.fast,

                    // 2. Prepare the hardware
                    ServoBlocker.INSTANCE.down,
                    Transfer.INSTANCE.off,
                    Intake.INSTANCE.off,

                    // 3. THE WAIT: The sequence stops here until the motor is actually fast enough
                    new WaitUntil(FlyNew.INSTANCE::isAtSpeed),

                    // 4. FIRE: This only runs AFTER isAtSpeed returns true
                    ServoBlocker.INSTANCE.up,
                    new Delay(1),
                    Transfer.INSTANCE.on,

                    // 5. SUCCESS: Keep things running for a moment to let the ball exit
                    new Delay(0.5),

                    // Optional: Turn intake back on or flywheel off
                    Intake.INSTANCE.on
            );

    public final Command TurretClose =
            new ParallelGroup(
                    Turret.INSTANCE.rotateAndReset
            );
}