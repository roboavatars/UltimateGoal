package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    public DcMotorEx intakeMotor;
    public DcMotorEx transferMotor;
    private Servo blockerServo;
    private Servo bumperLR;

    public static double intakePow = 1;
    public static double transferPow = 0.6;

    private double lastIntakePow = 0;
    private double lastTransferPow = 0;
    private double lastBlocker = 0;

    private double bumperLRPos;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = op.hardwareMap.get(DcMotorEx.class, "transfer");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        blockerServo = op.hardwareMap.get(Servo.class, "blocker");

        bumperLR = op.hardwareMap.get(Servo.class, "bumpers");

        if (!isAuto) {
            bumpersOut();
            blockerDown();
        } else {
            bumpersHome();
            blockerHome();
        }
        updateBumpers();

        op.telemetry.addData("Status", "Intake initialized");
    }

    // Intake Motors
    public void on() {
        setPower(intakePow, transferPow);
    }

    public void verticalOn() {
        setPower(0, 1);
    }

    public void reverse() {
        setPower(-1);
    }

    public void off() {
        setPower(0);
    }

    public void setPower(double power) {
        setPower(power, power);
    }

    public void setPower(double intakePower, double transferPower) {
        if (intakePower != lastIntakePow) {
            intakeMotor.setPower(intakePower);
            lastIntakePow = intakePower;
        }

        if (transferPower != lastTransferPow) {
            transferMotor.setPower(transferPower);
            lastTransferPow = transferPower;
        }
    }

    // Blocker
    public void blockerHome() {
        setBlocker(Constants.BLOCKER_HOME_POS);
    }

    public void blockerVert() {
        setBlocker(Constants.BLOCKER_VERTICAL_POS);
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

    // Bumpers
    public void bumpersLR(double position) {
        bumperLRPos = position;
    }

    public void bumpersHome() {
        bumpersLR(Constants.BUMPER_HOME_POS);
    }

    public void bumpersOut() {
        bumpersLR(Constants.BUMPER_OUT_POS);
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

        boolean leftFront = !inRange(leftFrontPos[0], leftFrontPos[1], buffer);
        boolean leftBack = !inRange(leftBackPos[0], leftBackPos[1], buffer);
        boolean rightFront = !inRange(rightFrontPos[0], rightFrontPos[1], buffer);
        boolean rightBack = !inRange(rightBackPos[0], rightBackPos[1], buffer);

        if (leftFront || leftBack || rightFront || rightBack) {
            bumpersLR(Constants.BUMPER_OUT_POS);
        } else {
            bumpersLR(Constants.BUMPER_HOME_POS);
        }
    }

    public void autoBlocker(double x, double y, double theta, double buffer) {
        double[] frontLeftPos = calculateCoordinates(x, y, theta, -9, 15);
        double[] frontBackPos = calculateCoordinates(x, y, theta, 9, 15);

        boolean frontLeft = !inRange(frontLeftPos[0], frontLeftPos[1], buffer);
        boolean frontRight = !inRange(frontBackPos[0], frontBackPos[1], buffer);

        if (frontLeft || frontRight) {
            blockerHome();
        } else {
            blockerDown();
        }
    }

    public void updateLRBumpers() {
        bumperLR.setPosition(bumperLRPos);
    }

    public void updateBumpers() {
        updateLRBumpers();
    }
}
