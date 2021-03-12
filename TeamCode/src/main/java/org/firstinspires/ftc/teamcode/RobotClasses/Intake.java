package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    public DcMotorEx intakeMotor;
    public DcMotorEx intakeMotor2;
    private Servo lStickServo;
    private Servo rStickServo;
    private Servo blockerServo;
    private boolean isAuto;

    private double leftStickPos;
    private double rightStickPos;

    private double lastIntakePow = 0;
    private double lastIntakePow2 = 0;
    private double lastBlocker = 0;

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor2 = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");

        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");
        blockerServo = op.hardwareMap.get(Servo.class, "blocker");

        this.isAuto = isAuto;
        op.telemetry.addData("Status", "Intake initialized");
    }

    public void on() {
        setPower(1);
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

            on = power != 0;
            if (power > 0) {
                forward = true;
                reverse = false;
            } else if (power < 0) {
                forward = false;
                reverse = true;
            }
            lastIntakePow = power;
        }
    }

    public void motor2Power(double power) {
        if (power != lastIntakePow2) {
            intakeMotor2.setPower(power);
            lastIntakePow2 = power;
        }
    }

    public void sticksHome() {
        leftStickPos = Constants.L_HOME_POS;
        rightStickPos = Constants.R_HOME_POS;
    }

    public void sticksHalf() {
        leftStickPos = Constants.L_HALF_POS;
        rightStickPos = Constants.R_HALF_POS;
    }

    public void leftHalf() {
        leftStickPos = Constants.L_HALF_POS;
    }

    public void sticksFourth() {
        leftStickPos = Constants.L_QUARTER_POS;
        rightStickPos = Constants.R_QUARTER_POS;
    }

    public void sticksOut() {
        leftStickPos = Constants.L_OUT_POS;
        rightStickPos = Constants.R_OUT_POS;
    }

    public void stickLeft(double position) {
        leftStickPos = position;
    }

    public void stickRight(double position) {
        rightStickPos = position;
    }

    public void autoSticks(double x, double y, double theta, double buffer) {
        double[] leftPos = new double[] {x - 33 * Math.sin(theta) + 7 * Math.cos(theta), y + 33 * Math.cos(theta) + 7 * Math.sin(theta)};
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

    public void sticksUpdate() {
        lStickServo.setPosition(leftStickPos);
        rStickServo.setPosition(rightStickPos);
    }

    public void blockerUp() {
        blockerServo.setPosition(Constants.BLOCKER_UP_POS);
    }

    public void blockerDown() {
        blockerServo.setPosition(Constants.BLOCKER_DOWN_POS);
    }

    public void setBlocker(double position) {
        if (position != lastBlocker) {
            blockerServo.setPosition(position * (Constants.BLOCKER_DOWN_POS - Constants.BLOCKER_UP_POS) + Constants.BLOCKER_UP_POS);
            lastBlocker = position;
        }
    }
}
