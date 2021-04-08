package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    public DcMotorEx intakeMotor;
    public DcMotorEx intakeMotor2;
    private Servo lStickServo;
    private Servo rStickServo;
    private Servo blockerServo;

    private double leftStickPos;
    private double rightStickPos;

    private double lastIntakePow = 0;
    private double lastBlocker = 0;

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor2 = op.hardwareMap.get(DcMotorEx.class, "intake2");

        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");
        blockerServo = op.hardwareMap.get(Servo.class, "blocker");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!isAuto) {
            sticksOut();
        } else {
            sticksHome();
        }
        updateSticks();

        blockerDown();

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void on() {
        setPower(1);
    }

    public void reverse() {
        setPower(-0.5);
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

    public void sticksHome() {
        stickLeft(Constants.L_HOME_POS);
        stickRight(Constants.R_HOME_POS);
    }

    public void sticksHalf() {
        stickLeft(Constants.L_HALF_POS);
        stickRight(Constants.R_HALF_POS);
    }

    public void sticksShoot() {
        stickLeft(Constants.L_SHOOT_POS);
        stickRight(Constants.R_SHOOT_POS);
    }

    public void sticksOut() {
        stickLeft(Constants.L_OUT_POS);
        stickRight(Constants.R_OUT_POS);
    }

    public void autoSticks(double x, double y, double theta, double buffer) {
        double[] leftPos = new double[] {x - 27 * Math.sin(theta) + 7 * Math.cos(theta), y + 27 * Math.cos(theta) + 7 * Math.sin(theta)};
        double[] rightPos = new double[] {x + 27 * Math.sin(theta) + 7 * Math.cos(theta), y - 27 * Math.cos(theta) + 7 * Math.sin(theta)};
        if (48 + buffer <= leftPos[0] && leftPos[0] <= 144 - buffer && 0 + buffer <= leftPos[1] && leftPos[1] <= 144 - buffer) {
            stickLeft(Constants.L_OUT_POS);
        } else {
            stickLeft(Constants.L_HALF_POS);
        }
        if (48 + buffer <= rightPos[0] && rightPos[0] <= 144 - buffer && 0 + buffer <= rightPos[1] && rightPos[1] <= 144 - buffer) {
            stickRight(Constants.R_OUT_POS);
        } else {
            stickRight(Constants.R_HALF_POS);
        }
    }

    public void stickLeft(double position) {
        leftStickPos = position;
    }

    public void stickRight(double position) {
        rightStickPos = position;
    }

    public void updateLeft() {
        lStickServo.setPosition(leftStickPos);
    }

    public void updateRight() {
        rStickServo.setPosition(rightStickPos);
    }

    public void updateSticks() {
        updateLeft();
        updateRight();
    }

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
}
