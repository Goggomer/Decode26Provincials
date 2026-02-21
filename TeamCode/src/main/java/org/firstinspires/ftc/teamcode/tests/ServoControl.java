package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Example")
public class ServoControl extends LinearOpMode {

    // Declare the servo object
    private Servo myServo;

    @Override
    public void runOpMode() {
        // Initialize the servo - "servo1" must match your Configuration on the Driver Station
        myServo = hardwareMap.get(Servo.class, "Stopper");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // If button A is pressed, move to position 0
            if (gamepad1.a) {
                myServo.setPosition(0.05);
            }
            // If button B is pressed, move to position 0.5
            else if (gamepad1.b) {
                myServo.setPosition(0.6);
            }

            // Display the current position for debugging
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}