package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RGBSub {

    private Servo rgb;

    private ElapsedTime timer = new ElapsedTime();

    // =========================
    // COLOR POSITIONS (TUNE IF NEEDED)
    // =========================
    public static final double RED     = 0.279;
    public static final double ORANGE  = 0.333;
    public static final double YELLOW  = 0.388;
    public static final double GREEN   = 0.5;
    public static final double BLUE    = 0.611;
    public static final double PURPLE  = 0.722;
    public static final double WHITE   = 1.0;
    public static final double OFF     = 0.0;

    private final double[] solidColors = {
            RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE, WHITE
    };

    private int solidIndex = 0;

    // =========================
    // EFFECT STATES
    // =========================
    private enum Mode {
        SOLID,
        BLINK,
        GRADIENT,
        RAINBOW
    }

    private Mode currentMode = Mode.SOLID;

    private double currentColor = RED;

    private double blinkInterval = 0.5;
    private boolean blinkState = false;

    private double gradientStart;
    private double gradientEnd;
    private double gradientSpeed = 1.5;

    public RGBSub(HardwareMap hardwareMap) {
        rgb = hardwareMap.get(Servo.class, "rgb");
        rgb.setPosition(RED);
    }

    // =========================
    // SOLID COLOR
    // =========================
    public void setSolid(double color) {
        currentMode = Mode.SOLID;
        currentColor = color;
        rgb.setPosition(color);
    }

    public void nextSolidColor() {
        solidIndex++;
        if (solidIndex >= solidColors.length) solidIndex = 0;
        setSolid(solidColors[solidIndex]);
    }

    public void previousSolidColor() {
        solidIndex--;
        if (solidIndex < 0) solidIndex = solidColors.length - 1;
        setSolid(solidColors[solidIndex]);
    }

    // =========================
    // BLINK
    // =========================
    public void startBlink(double color, double intervalSeconds) {
        currentMode = Mode.BLINK;
        currentColor = color;
        blinkInterval = intervalSeconds;
        timer.reset();
    }

    // =========================
    // GRADIENT
    // =========================
    public void startGradient(double start, double end, double speed) {
        currentMode = Mode.GRADIENT;
        gradientStart = start;
        gradientEnd = end;
        gradientSpeed = speed;
        timer.reset();
    }

    // =========================
    // RAINBOW
    // =========================
    public void startRainbow(double speed) {
        currentMode = Mode.RAINBOW;
        gradientSpeed = speed;
        timer.reset();
    }

    // =========================
    // UPDATE (MUST CALL IN LOOP)
    // =========================
    public void update() {

        switch (currentMode) {

            case SOLID:
                break;

            case BLINK:
                if (timer.seconds() > blinkInterval) {
                    blinkState = !blinkState;
                    rgb.setPosition(blinkState ? currentColor : OFF);
                    timer.reset();
                }
                break;

            case GRADIENT:
                double progress = (Math.sin(timer.seconds() * gradientSpeed) + 1) / 2;
                double position = gradientStart +
                        (gradientEnd - gradientStart) * progress;
                rgb.setPosition(position);
                break;

            case RAINBOW:
                double rainbowPos = (Math.sin(timer.seconds() * gradientSpeed) + 1) / 2;
                rgb.setPosition(rainbowPos);
                break;
        }
    }

    public String getModeName() {
        return currentMode.name();
    }
}