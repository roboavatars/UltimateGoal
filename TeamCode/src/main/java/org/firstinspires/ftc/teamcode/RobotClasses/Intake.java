package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    public DcMotorEx intakeMotor;
    public DcMotorEx intakeMotor2;
    private Servo blockerServo;
    private Servo stackServo;
    private Servo bumperLeft;
    private Servo bumperRight;

    private double lastIntakePow = 0;
    private double lastBlocker = 0;

    private double leftBumperPos;
    private double rightBumperPos;

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor2 = op.hardwareMap.get(DcMotorEx.class, "intake2");

        blockerServo = op.hardwareMap.get(Servo.class, "blocker");
        stackServo = op.hardwareMap.get(Servo.class, "stackServo");

        bumperLeft = op.hardwareMap.get(Servo.class, "bumperLeft");
        bumperRight = op.hardwareMap.get(Servo.class, "bumperRight");

//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!isAuto) {
            bumpersOut();
            blockerDown();
        } else {
            bumpersHome();
            blockerHome();
        }
        stackHome();
        updateBumpers();

        op.telemetry.addData("Status", "Intake initialized");
    }

    // Intake Motors
    public void on() {
        setPower(1);
    }

    public void verticalOn() {
        intakeMotor.setPower(0);
        intakeMotor2.setPower(1);
        lastIntakePow = 2;
    }

    public void reverse() {
        setPower(-1);
    }

    public void off() {
        setPower(0);
    }

    public void setPower(double power) {
        if (power != lastIntakePow) {
            intakeMotor.setPower(power);
            intakeMotor2.setPower(power);

            on = power != 0;
            forward = power > 0;
            reverse = power < 0;
            lastIntakePow = power;
        }
    }

    // Blocker
    public void blockerHome() {
        setBlocker(Constants.BLOCKER_HOME_POS);
    }

    public void blockerUp() {
        setBlocker(Constants.BLOCKER_UP_POS);
    }

    public void blockerDown() {
        setBlocker(Constants.BLOCKER_DOWN_POS);
    }

    public void setBlocker(double position) {
        if (position != lastBlocker) {
            blockerServo.setPosition(position);
            lastBlocker = position;
        }
    }

    // Stack Servo
    public void stackHome() {
        stackServo.setPosition(Constants.STACK_HOME_POS);
    }

    public void stackOut() {
        stackServo.setPosition(Constants.STACK_OUT_POS);
    }

    // Bumpers
    public void bumperLeft(double position) {
        leftBumperPos = position;
    }

    public void bumperRight(double position) {
        rightBumperPos = position;
    }

    public void bumpersHome() {
        bumperLeft(Constants.BUMPER_LEFT_HOME_POS);
        bumperRight(Constants.BUMPER_RIGHT_HOME_POS);
    }

    public void bumpersOut() {
        bumperLeft(Constants.BUMPER_LEFT_OUT_POS);
        bumperRight(Constants.BUMPER_RIGHT_OUT_POS);
    }

    private double[] calculateCoordinates(double x, double y, double theta, double dx, double dy) {
        return new double[] {x + dx * Math.sin(theta) + dy * Math.cos(theta), y - dy * Math.cos(theta) + dy * Math.sin(theta)};
    }

    private boolean inRange(double x, double y, double buffer) {
        return buffer <= x && x <= 144 - buffer && buffer <= y && y <= 144 - buffer;
    }

    public void autoBumpers(double x, double y, double theta, double buffer) {
        double[] leftFrontPos = calculateCoordinates(x, y, theta, -15, 9);
        double[] leftBackPos = calculateCoordinates(x, y, theta, -15, -9);
        double[] rightFrontPos = calculateCoordinates(x, y, theta, 15, 6);
        double[] rightBackPos = calculateCoordinates(x, y, theta, 15, -9);

        boolean leftFront = inRange(leftFrontPos[0], leftFrontPos[1], buffer);
        boolean leftBack = inRange(leftBackPos[0], leftBackPos[1], buffer);
        boolean rightFront = inRange(rightFrontPos[0], rightFrontPos[1], buffer);
        boolean rightBack = inRange(rightBackPos[0], rightBackPos[1], buffer);

        if (leftFront || leftBack) {
            bumperLeft(Constants.BUMPER_LEFT_HOME_POS);
        } else {
            bumperLeft(Constants.BUMPER_LEFT_OUT_POS);
        }

        if (rightFront || rightBack) {
            bumperRight(Constants.BUMPER_RIGHT_HOME_POS);
        } else {
            bumperRight(Constants.BUMPER_RIGHT_OUT_POS);
        }
    }

    public void updateLeftBumper() {
        bumperLeft.setPosition(leftBumperPos);
    }

    public void updateRightBumper() {
        bumperRight.setPosition(rightBumperPos);
    }

    public void updateBumpers() {
        updateLeftBumper();
        updateRightBumper();
    }
}
