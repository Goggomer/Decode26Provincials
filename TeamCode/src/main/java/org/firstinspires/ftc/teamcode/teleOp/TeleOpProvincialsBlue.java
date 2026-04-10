package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class TeleOpProvincialsBlue extends LinearOpMode {
    RobotHardwareProvincialsBlue teleRobot = new RobotHardwareProvincialsBlue();
    boolean superModeToggled = false, shooterOn = false, distanceSensorOn = true, detected = false, detected2 = false;
    boolean stateHigh = false;
    boolean xWasPressed = false, yWasPressed = false, aWasPressed = false, bWasPressed = false, leftBumper2WasPressed = false, back2WasPressed = false;
    boolean x2WasPressed = false, b2WasPressed = false;
    boolean intakeOn = false, transferOn = false, resettingSpinnerZeroOn = true;
    boolean isResettingToZero = false;
    double currentTargetTPS = 0, currentP = 0, currentF = 0, currentRPM = 0;
    boolean isAutoAim = false, manualTogglePreviouslyPressed = false;
    public double manualTargetDegree = 0.0;
    public final double turretSpeed = 1.5;
    public RGBSub rgb;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleRobot.init(hardwareMap);
        rgb = new RGBSub(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            // Field Centric Drive via Control Hub IMU
            double y = -gamepad1.left_stick_y, x = gamepad1.left_stick_x, rx = gamepad1.right_stick_x;
            if (gamepad1.options) teleRobot.imu.resetYaw();
            double botHeading = teleRobot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            teleRobot.frontLeftMotor.setPower((rotY + rotX + rx) / den);
            teleRobot.backLeftMotor.setPower((rotY - rotX + rx) / den);
            teleRobot.frontRightMotor.setPower((rotY - rotX - rx) / den);
            teleRobot.backRightMotor.setPower((rotY + rotX - rx) / den);

            allControls();
            handleTurretLogic();
            rgb.update();

            telemetry.addData("Mode", isAutoAim ? "AUTO AIM (Limelight)" : "MANUAL (D-Pad)");
            telemetry.addData("Stopper", teleRobot.stopperCovering ? "120°" : "0°");
            telemetry.addData("Turret Angle", teleRobot.getCurrentTurretDegrees());
            telemetry.addData("Shooter RPM", teleRobot.getActualRPM());
            telemetry.addData("Detected:", detected);
            telemetry.addData("DistanceSensorOn:", distanceSensorOn);
            telemetry.addData("Target X (Raw)", "%.2f°", RobotHardwareProvincialsBlue.teleRawError) ;
            telemetry.addData("Target X (Filtered)", "%.2f°", RobotHardwareProvincialsBlue.filteredError);
            telemetry.update();
        }
        teleRobot.limelight.stop();
    }

    private void handleTurretLogic() {
        boolean manualTogglePressed = gamepad2.a;
        if (manualTogglePressed && !manualTogglePreviouslyPressed) {
            isAutoAim = !isAutoAim;
            if (!isAutoAim) {
                manualTargetDegree = teleRobot.getCurrentTurretDegrees();
            }
        }
        manualTogglePreviouslyPressed = manualTogglePressed;

        if (isAutoAim) {
            teleRobot.turretSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
            teleRobot.updateLimelightTracking();
        } else {
            teleRobot.turretSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
            handleManualTurretControls();
        }
    }

    private void handleManualTurretControls() {
        if (teleRobot.turretSpinner.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            teleRobot.turretSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            teleRobot.turretSpinner.setPower(1.0);
        }
        if (gamepad2.left_bumper && resettingSpinnerZeroOn && !isResettingToZero) {
            manualTargetDegree = 119;
            isResettingToZero = true;
            resettingSpinnerZeroOn = false;
        } // LEFT BUMPER for FAR
        if (gamepad2.back && resettingSpinnerZeroOn && !isResettingToZero) {
            manualTargetDegree = 138;
            isResettingToZero = true;
            resettingSpinnerZeroOn = false;
        } // BACK for CLOSE
        if (isResettingToZero) {
            if (Math.abs(teleRobot.getCurrentTurretDegrees() - manualTargetDegree) < 3.0) {
                teleRobot.turretSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                manualTargetDegree = 0;
                isResettingToZero = false;
            } else {
                teleRobot.setSpinnerAngle(manualTargetDegree);
            }
        } else {
            if (gamepad2.dpad_right) manualTargetDegree -= turretSpeed;
            if (gamepad2.dpad_left) manualTargetDegree += turretSpeed;
            if (gamepad2.y) manualTargetDegree = 67;
            if (gamepad2.b) manualTargetDegree = 50;
            manualTargetDegree = Math.max(-180, Math.min(180, manualTargetDegree));
            teleRobot.setSpinnerAngle(manualTargetDegree);
        }
    }

    private void allControls() {

        // Shooting Time
        if (gamepad2.x && !x2WasPressed) {
            superModeToggled = !superModeToggled;
        }
        x2WasPressed = gamepad2.x;

        if (gamepad1.y && !yWasPressed) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                currentTargetTPS = RobotHardwareProvincialsBlue.targetTPSFar1; // Far1
                currentP = RobotHardwareProvincialsBlue.PFar1;
                currentF = RobotHardwareProvincialsBlue.FFar1;
                currentRPM = RobotHardwareProvincialsBlue.targetRPMFar1;
            }
        }
        yWasPressed = gamepad1.y;

        if (gamepad1.a && !aWasPressed) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                currentTargetTPS = RobotHardwareProvincialsBlue.targetTPSFar2; // Far2
                currentP = RobotHardwareProvincialsBlue.PFar2;
                currentF = RobotHardwareProvincialsBlue.FFar2;
                currentRPM = RobotHardwareProvincialsBlue.targetRPMFar2;
            }
        }
        aWasPressed = gamepad1.a;

        if (gamepad1.x && !xWasPressed) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                currentTargetTPS = RobotHardwareProvincialsBlue.targetTPSClose1; //Close1
                currentP = RobotHardwareProvincialsBlue.PClose1;
                currentF = RobotHardwareProvincialsBlue.FClose1;
                currentRPM = RobotHardwareProvincialsBlue.targetRPMClose1;
            }
        }
        xWasPressed = gamepad1.x;

        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                currentTargetTPS = RobotHardwareProvincialsBlue.targetTPSClose2; //Close2
                currentP = RobotHardwareProvincialsBlue.PClose2;
                currentF = RobotHardwareProvincialsBlue.FClose2;
                currentRPM = RobotHardwareProvincialsBlue.targetRPMClose2;
            }
        }
        bWasPressed = gamepad1.b;

        if (shooterOn) {
            teleRobot.Flywheel.setVelocityPIDFCoefficients(currentP, 0, 0, currentF);
            teleRobot.Flywheel.setVelocity(currentTargetTPS);
            teleRobot.stopper.setPosition(0.6);
            if (Math.abs(teleRobot.getActualRPM() - currentRPM) <= 150) {
                rgb.setSolid(RGBSub.GREEN);
            } else {
                rgb.setSolid(RGBSub.RED);
            }
        } else {
            teleRobot.Flywheel.setPower(0);
            teleRobot.stopper.setPosition(0.05);
            rgb.setSolid(RGBSub.OFF);
        }

        // Intake and Transfer
        double intakePower = 0;
        double transferPower = 0;

        if (superModeToggled) intakePower = 0.8;
        else if (gamepad2.left_trigger > 0.5) intakePower = -0.5; // Reversing
        else if (gamepad1.left_trigger > 0.5) intakePower = 0.9; // Normal Intaking
        else intakePower = 0; // Nil

        if (superModeToggled) {
            transferPower = 1.0; // Shooting
        } else if (gamepad2.right_bumper) { // Reversing and Slapping the Ball to move down
            transferPower = -1.0;
        } else if (gamepad2.right_trigger > 0.5) {
            transferPower = 0.85; // Manual Transfer Control from Gamepad2
        } else {
            transferPower = 0; // Nil
        }

        teleRobot.intake.setPower(intakePower);
        teleRobot.transfer.setPower(transferPower);

        intakeOn = Math.abs(intakePower) > 0;
        transferOn = Math.abs(transferPower) > 0;
    }
}