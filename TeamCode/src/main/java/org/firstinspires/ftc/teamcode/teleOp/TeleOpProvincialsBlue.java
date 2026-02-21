package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.REVDistanceSensorSub;
import org.firstinspires.ftc.teamcode.teleOp.RobotHardwareProvincialsBlue;
import org.firstinspires.ftc.teamcode.teleOp.RGBSub;
import org.firstinspires.ftc.teamcode.teleOp.ConfigLED;
import org.firstinspires.ftc.teamcode.teleOp.FieldCentricDrive;

@Config
@TeleOp
public class TeleOpProvincialsBlue extends LinearOpMode {
    RobotHardwareProvincialsBlue teleRobot = new RobotHardwareProvincialsBlue();
    ConfigLED leddy = new ConfigLED();
    REVDistanceSensorSub revy = new REVDistanceSensorSub();
    boolean superModeToggled = false, shooterOn = false, distanceSensorOn = true, detected = false, detected2 = false;
    boolean stateHigh = false;
    boolean xWasPressed = false, yWasPressed = false, aWasPressed = false, bWasPressed = false;
    boolean x2WasPressed = false, b2WasPressed = false;
    boolean intakeOn = false, transferOn = false;
    double currentTargetTPS = 0, currentP = 0, currentF = 0, currentRPM = 0;
    boolean isAutoAim = false, manualTogglePreviouslyPressed = false;
    public double manualTargetDegree = 0.0;
    public final double turretSpeed = 3.0;
    public RGBSub rgb;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleRobot.init(hardwareMap);
        rgb = new RGBSub(hardwareMap);
        leddy.init(hardwareMap);
        revy.init(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, gamepad1, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.options) {
                drive.resetHeading();
            }
            drive.update();

            allControls();
            handleTurretLogic();
            rgb.update();

            telemetry.addData("Mode", isAutoAim ? "AUTO AIM (Limelight)" : "MANUAL (D-Pad)");
            telemetry.addData("Stopper", teleRobot.stopperCovering ? "120째" : "0째");
            telemetry.addData("Turret Angle", teleRobot.getCurrentTurretDegrees());
            telemetry.addData("Shooter RPM", teleRobot.getActualRPM());
            telemetry.addData("Raw (HIGH/LOW)", stateHigh = teleRobot.getDistanceState());
            telemetry.addData("Detected:", detected);
            telemetry.addData("DistanceSensorOn:", distanceSensorOn);
            telemetry.addData("Target X (Raw)", "%.2f째", RobotHardwareProvincialsBlue.teleRawError) ;
            telemetry.addData("Target X (Filtered)", "%.2f째", RobotHardwareProvincialsBlue.filteredError);
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
        if (gamepad2.dpad_right) manualTargetDegree -= turretSpeed;
        if (gamepad2.dpad_left) manualTargetDegree += turretSpeed;
        if (gamepad2.y) manualTargetDegree = -90;
        if (gamepad2.b) manualTargetDegree = 90;
        manualTargetDegree = Math.max(-180, Math.min(180, manualTargetDegree));
        teleRobot.setSpinnerAngle(manualTargetDegree);
    }

    private void allControls() {
        /* boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        if (leftTriggerPressed && !leftTriggerPreviouslyPressed) {
            intakeToggled = !intakeToggled;
        }
        leftTriggerPreviouslyPressed = leftTriggerPressed; */

        // Distance Sensor Code
        stateHigh = teleRobot.getDistanceState();
        /* if (gamepad2.b && !b2WasPressed){
            distanceSensorOn = !distanceSensorOn;
        }
        b2WasPressed = gamepad2.b; */

        if (distanceSensorOn){
            detected = stateHigh;
        } else{
            detected = false;
        }
        detected2 = (revy.sensorDistance.getDistance(DistanceUnit.INCH) > 2) && (revy.sensorDistance.getDistance(DistanceUnit.INCH) < 8.5);

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
            teleRobot.turretFlywheel.setVelocityPIDFCoefficients(currentP, 0, 0, currentF);
            teleRobot.turretFlywheel.setVelocity(currentTargetTPS);
            teleRobot.stopper.setPosition(0.6);
            if (Math.abs(teleRobot.getActualRPM() - currentRPM) <= 150) {
                rgb.setSolid(RGBSub.GREEN);
            } else {
                rgb.setSolid(RGBSub.RED);
            }
        } else {
            teleRobot.turretFlywheel.setPower(0);
            teleRobot.stopper.setPosition(0.05);
            if (detected && detected2) {
                rgb.setSolid(RGBSub.BLUE);
            } else if (detected){
                rgb.setSolid(RGBSub.YELLOW);
            } else if (detected2){
                rgb.setSolid(RGBSub.RED);
            } else {
                rgb.setSolid(RGBSub.OFF);
            }
        }

        // Intake and Transfer
        double intakePower = 0;
        double transferPower = 0;

        if (superModeToggled) intakePower = 0.8; // Shooting - this could be changed in a range from 0.6-1.0 just in case balls are releasing too early
        else if (gamepad2.left_bumper) intakePower = -0.5; // Reversing
            // else if (detected) intakePower = 0.55; // Slowing down intake when the distance sensor has got a ball
        else if (gamepad1.left_trigger > 0.5) intakePower = 0.9; // Normal Intaking
        else intakePower = 0; // Nil

        if (superModeToggled) transferPower = 1.0; // Shooting
        else if (gamepad2.right_bumper){ // Reversing and Slapping the Ball to move down
            transferPower = -1.0;
            teleRobot.stopper.setPosition(0.0);
            sleep(200);
            teleRobot.stopper.setPosition(0.05);
        }
        else if (gamepad2.right_trigger > 0.5) transferPower = 0.85; // Manual Transfer Control from Gamepad2
            // else if (detected) transferPower = 0.0; // Distance Sensor stops transfer when it sees first ball
            // else if (gamepad1.left_trigger > 0.5 && distanceSensorOn) transferPower = 0.9; // Normal Intaking
        else transferPower = 0; // Nil

        teleRobot.intake.setPower(intakePower);
        teleRobot.transfer.setPower(transferPower);

        intakeOn = Math.abs(intakePower) > 0;
        transferOn = Math.abs(transferPower) > 0;
        leddy.setIntakeLed(intakeOn);
        leddy.setTransferLed(transferOn);

        telemetry.update();
    }
}