package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    private DcMotorEx intakeMotor;
    private Servo lStickServo;
    private Servo rStickServo;
    private Servo blockerServo;
    private boolean isAuto;

    private double lastIntakePow = 0;

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");
        blockerServo = op.hardwareMap.get(Servo.class, "blocker");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        closeBlocker();

        this.isAuto = isAuto;
        op.telemetry.addData("Status", "Intake initialized");
    }

    public void intakeOn() {
        setPower(1);
    }

    public void intakeRev() {
        setPower(-1);
    }

    public void intakeOff() {
        setPower(0);
    }

    public void setPower(double power) {
        if (power != lastIntakePow) {
            intakeMotor.setPower(power);
            on = power != 0;
            if (power > 0) { forward = true; }
            else if (power < 0) { reverse = true; }
            lastIntakePow = power;
        }
    }

    public void sticksHome() {
        if (!isAuto) {
            lStickServo.setPosition(Constants.L_HOME_POS);
            rStickServo.setPosition(Constants.R_HOME_POS);
        }
    }

    public void sticksHalf() {
        lStickServo.setPosition(Constants.L_HALF_POS);
        rStickServo.setPosition(Constants.R_HALF_POS);
    }

    public void sticksFourth() {
        lStickServo.setPosition(Constants.L_QUARTER_POS);
        rStickServo.setPosition(Constants.R_QUARTER_POS);
    }

    public void sticksOut() {
        lStickServo.setPosition(Constants.L_OUT_POS);
        rStickServo.setPosition(Constants.R_OUT_POS);
    }

    public void stickLeft(double position) {
        lStickServo.setPosition(position);
    }

    public void stickRight(double position) {
        rStickServo.setPosition(position);
    }

    public void closeBlocker() {
        blockerServo.setPosition(Constants.BLOCKER_CLOSE_POS);
    }

    public void openBlocker() {
        blockerServo.setPosition(Constants.BLOCKER_OPEN_POS);
    }

    public void setBlocker(double position) {
        blockerServo.setPosition(position * (Constants.BLOCKER_OPEN_POS - Constants.BLOCKER_CLOSE_POS) + Constants.BLOCKER_CLOSE_POS);
    }
}
