package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@Config
public class RobotHardwareProvincialsRed {

    // ---------------- Intake & Transfer & Distance Sensor & Servo ----------------
    public DcMotor intake, transfer;
    public DigitalChannel laserInput;
    public Servo stopper;

    // ---------------- Turret Flywheel ----------------
    public DcMotorEx turretFlywheel;
    public static double PClose1 = 40.0, FClose1 = 14.25; // Closer too shooting area (Press X during TeleOp)
    public static double PClose2 = 40.0, FClose2 = 14.25; // Sujaan Auto (Press B during TeleOp)
    public static double PFar1 = 50.0, FFar1 = 14.075; // Backed up Far (Press Y during TeleOp)
    public static double PFar2 = 50.0, FFar2 = 13.80; // Sujaan Auto Placement Far (Press A during TeleOp)
    public static double I = 0.0, D = 0.0;
    public static double targetRPMClose1 = 2600, targetRPMClose2  = 2500, targetRPMFar1 = 3200, targetRPMFar2 = 3100;
    public static double targetTPSClose1 = 0, targetTPSClose2  = 0, targetTPSFar1 = 3400, targetTPSFar2 = 3300;
    public static double shooterGearRatio = 1.0;
    public static double TICKS_PER_REV = 28.0 * shooterGearRatio;

    // ---------------- Turret Tracker/Spinner ----------------

    public DcMotorEx turretSpinner;
    public Limelight3A limelight;

    public static double Kp = 0.02, Ki = 0, Kd = 0.02;
    public static double lowPassGain = 0.0, deadband = 0.0, kStatic = 0.0045;
    public static double errorSum = 0, lastError = 0, filteredError = 0;
    public static double teleRawError = 0.0;
    public static double MAX_TURRET_DEGREES = 180.0, MIN_TURRET_DEGREES = -180.0;

    // ---------------- SPINNER Constants ----------------
    public final double spinnerEncoderPPR = 145.1;
    public final double spinnerGearRatio = 6.0;
    public final double spinnerTicksPerDegree = (spinnerEncoderPPR * spinnerGearRatio) / 360.0;

    // ---------------- Auto Booleans ----------------
    public boolean shooterOnAuto = false;
    public boolean stopperCovering = true;

    // ---------------- Hardware & ConfigFile ----------------

    public void init(HardwareMap hwMap) {

        // Intake/Transfer
        intake = hwMap.get(DcMotor.class, "Intake");
        transfer = hwMap.get(DcMotor.class, "Transfer");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        laserInput = hwMap.get(DigitalChannel.class, "laserDigitalInput");
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        stopper = hwMap.get(Servo.class, "Stopper");
        stopper.setPosition(0.30);

        // Turret Flywheel
        turretFlywheel = hwMap.get(DcMotorEx.class, "Shooter");
        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // ---
        turretFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        targetTPSClose1 = targetRPMClose1 * TICKS_PER_REV / 60.0;
        targetTPSClose2  = targetRPMClose2  * TICKS_PER_REV / 60.0;
        targetTPSFar1 = targetRPMFar1 * TICKS_PER_REV / 60.0;
        targetTPSFar2  = targetRPMFar2  * TICKS_PER_REV / 60.0;


        // Turret Tracker/Spinner (MAKE SURE TURRET IS CENTERED ON INIT)
        turretSpinner = hwMap.get(DcMotorEx.class, "Turret");
        turretSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSpinner.setTargetPosition(0);
        turretSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        turretSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretSpinner.setPower(0);

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    // Distance Sensor Function
    public boolean getDistanceState(){
        return laserInput.getState();
    }

    // Shooter RPM Function

    public double getActualRPM() {
        return (turretFlywheel.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    // ---------------- Turret Tracker/Spinner ----------------
    public void updateLimelightTracking() {
        if (turretSpinner.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double rawError = result.getTx();

            // PID Math
            filteredError = (lowPassGain * filteredError) + ((1 - lowPassGain) * rawError);
            if (Math.abs(filteredError) < 3.0) errorSum += filteredError;
            else errorSum = 0;

            double derivative = filteredError - lastError;
            double motorPower = (filteredError * Kp) + (errorSum * Ki) + (derivative * Kd);

            // Friction and Deadband logic
            if (Math.abs(filteredError) <= deadband) {
                motorPower = 0;
                errorSum = 0;
            } else {
                motorPower += Math.signum(filteredError) * kStatic;
            }

            double currentDegrees = getCurrentTurretDegrees();

            if (motorPower > 0 && currentDegrees > MAX_TURRET_DEGREES) {
                motorPower = 0;
            }
            else if (motorPower < 0 && currentDegrees < MIN_TURRET_DEGREES) {
                motorPower = 0;
            }

            // Power Cap and Execution
            motorPower = Math.max(-1.0, Math.min(1.0, motorPower));
            turretSpinner.setPower(motorPower);
            lastError = filteredError;

            teleRawError = rawError;
        } else {
            turretSpinner.setPower(0);
            errorSum = 0;
            lastError = 0;
        }
    }
    public void setSpinnerAngle(double degrees) {
        // Clamping the input to ensure it stays within 180 degrees both ways
        double clampedDegrees = Math.max(-180, Math.min(180, degrees));

        // Calculate ticks: 145.1 / 360 = ~0.4 ticks per degree
        int targetTicks = (int) (clampedDegrees * spinnerTicksPerDegree);

        turretSpinner.setTargetPosition(targetTicks);
        turretSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretSpinner.setPower(1.0);
    }

    public double getFilteredError() { return filteredError; }

    public double getCurrentTurretDegrees() {
        return turretSpinner.getCurrentPosition() / spinnerTicksPerDegree;
    }

    // ---------------- Autonomous Functions - Intake/Transfer & Distance Sensor ----------------
    public void intakeForward() {
        intake.setPower(1.0);
    }
    public void intakeStop() {
        intake.setPower(0);
    }
    public void transferForward1() {
        transfer.setPower(1.0);
    }

    public void transferForward2() {
        transfer.setPower(0.75);
    }
    public void transferStop() {
        transfer.setPower(0);
    }

    // ---------------- Autonomous Functions - Flywheel Shooter ----------------
    public void setShooterClose1RPM() {
        turretFlywheel.setVelocityPIDFCoefficients(PClose1, I, D, FClose1);
        turretFlywheel.setVelocity(targetTPSClose1);
        shooterOnAuto = true;
    }

    public void setShooterClose2RPM() {
        turretFlywheel.setVelocityPIDFCoefficients(PClose2, I, D, FClose2);
        turretFlywheel.setVelocity(targetTPSClose2);
        shooterOnAuto = true;
    }

    public void setShooterFar1RPM() {
        turretFlywheel.setVelocityPIDFCoefficients(PFar1, I, D, FFar1);
        turretFlywheel.setVelocity(targetTPSFar1);
        shooterOnAuto = true;
    }

    public void setShooterFar2RPM() {
        turretFlywheel.setVelocityPIDFCoefficients(PFar2, I, D, FFar2);
        turretFlywheel.setVelocity(targetTPSFar2);
        shooterOnAuto = true;
    }

    public void stopShooter() {
        turretFlywheel.setVelocity(0);
        shooterOnAuto = false;
    }
}