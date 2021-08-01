package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private Servo blockerServo;
    private Servo bumperServo;

    private double lastIntakePow = 10;
    private double lastTransferPow = 10;
    private double lastBlockerPos = 10;
    private double lastBumperPos = 10;
    public boolean bumperReverse = false;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = op.hardwareMap.get(DcMotorEx.class, "transfer");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        blockerServo = op.hardwareMap.get(Servo.class, "blocker");
        bumperServo = op.hardwareMap.get(Servo.class, "bumpers");

        if (!isAuto) {
            bumpersOut();
            blockerVert();
        } else {
            bumpersHome();
            blockerHome();
        }

        op.telemetry.addData("Status", "Intake initialized");
    }

    // Intake Motors
    public void on() {
        setPower(Constants.INTAKE_POWER, Constants.TRANSFER_POWER);
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
        setBlockerPos(Constants.BLOCKER_HOME_POS);
    }

    public void blockerVert() {
        setBlockerPos(Constants.BLOCKER_VERTICAL_POS);
    }

    public void setBlockerPos(double position) {
        if (position != lastBlockerPos) {
            blockerServo.setPosition(position);
            lastBlockerPos = position;
        }
    }

    // Bumpers
    public void setBumperPos(double position) {
        if (position != lastBumperPos) {
            lastBumperPos = position;
            bumperServo.setPosition(lastBumperPos);
        }
    }

    public void bumpersHome() {
        if (bumperReverse) {
            setBumperPos(Constants.BUMPER_OUT_POS);
        } else {
            setBumperPos(Constants.BUMPER_HOME_POS);
        }
    }

    public void bumpersOut() {
        if (bumperReverse) {
            setBumperPos(Constants.BUMPER_HOME_POS);
        } else {
            setBumperPos(Constants.BUMPER_OUT_POS);
        }
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

        if (!leftFront || !leftBack || !rightFront || !rightBack) {
            bumpersHome();
        } else {
            bumpersOut();
        }
    }
}
