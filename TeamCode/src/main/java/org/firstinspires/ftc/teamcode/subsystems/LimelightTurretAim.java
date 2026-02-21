package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight 3A Turret Aim")
public class LimelightTurretAim extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turretMotor;

    // --- CONFIGURATION ---
    // Update these coordinates based on the official DeCode field manual or your specific needs.
    // For now, I'm using placeholder values (in Meters).
    // ID 20 is usually a Blue Alliance goal.
    // (0,0) is field center. Y+ is usually towards the 'back' wall, X+ towards the 'red' side.
    private static final double TARGET_X = 1.2;  // Example: 1.2 meters forward
    private static final double TARGET_Y = 0.5;  // Example: 0.5 meters left

    // Turret Control PID Constants (Tune these!)
    private static final double TURRET_kP = 0.02; // Proportional gain
    private static final double MAX_POWER = 0.5;  // Safety limit

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Hardware Initialization
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Important: Set the poll rate. 100Hz is fast enough for smooth aiming.
        limelight.setPollRateHz(100);

        telemetry.setMsTransmissionInterval(11); // Faster telemetry for debugging
        limelight.start(); // Start the camera

        // Switch to pipeline 0 (make sure your AprilTag pipeline is set to 0 in the web UI)
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 2. Get the latest data from Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // 3. Get Robot (Camera) Position
                // We use BotPose. Since the camera is on the turret, this 'Yaw' is
                // effectively the Turret's global heading.
                Pose3D botPose = result.getBotpose();

                if (botPose != null) {
                    double currentX = botPose.getPosition().x;
                    double currentY = botPose.getPosition().y;

                    // Limelight Yaw is usually in degrees (-180 to 180)
                    double currentTurretHeading = botPose.getOrientation().getYaw();

                    // 4. The Math: Calculate angle to target
                    // atan2(deltaY, deltaX) gives the absolute field angle we WANT to face
                    double deltaX = TARGET_X - currentX;
                    double deltaY = TARGET_Y - currentY;

                    // Math.atan2 returns radians, convert to degrees
                    double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));

                    // 5. Calculate the error (How much do we need to turn?)
                    double headingError = targetHeading - currentTurretHeading;

                    // 6. Normalize error to smallest turn (-180 to 180)
                    while (headingError > 180)  headingError -= 360;
                    while (headingError <= -180) headingError += 360;

                    // 7. Apply Power to Turret
                    // If error is positive (target is left), turn left.
                    double power = headingError * TURRET_kP;

                    // Clamp power to safety limits
                    power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

                    // Deadband (stop jittering if we are close enough, e.g., 1 degree)
                    if (Math.abs(headingError) < 1.0) {
                        power = 0;
                    }

                    // You might need to negate 'power' depending on your motor wiring!
                    turretMotor.setPower(power);

                    // Telemetry for Debugging
                    telemetry.addData("Target", "ID 20");
                    telemetry.addData("Pos", "X: %.2f, Y: %.2f", currentX, currentY);
                    telemetry.addData("Heading", "Curr: %.1f, Targ: %.1f", currentTurretHeading, targetHeading);
                    telemetry.addData("Error", "%.2f deg", headingError);
                    telemetry.addData("Motor Pwr", "%.2f", power);
                }
            } else {
                // Failsafe: If no tag detected, stop turret or go to manual control
                turretMotor.setPower(0);
                telemetry.addData("Limelight", "No Target / No Pose");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}