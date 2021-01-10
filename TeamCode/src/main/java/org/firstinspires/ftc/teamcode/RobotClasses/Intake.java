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

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");
        blockerServo = op.hardwareMap.get(Servo.class, "blocker");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        closeBlocker();

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void intakeOn() {
        intakeMotor.setPower(1);
        on = true;
        reverse = false;
        forward = true;
    }

    public void intakeRev() {
        intakeMotor.setPower(-1);
        on = true;
        reverse = true;
        forward = false;
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
        on = false;
        reverse = false;
        forward = false;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
        on = power != 0;
    }

    public void sticksHome() {
        lStickServo.setPosition(Constants.L_HOME_POS);
        rStickServo.setPosition(Constants.R_HOME_POS);
    }

    public void sticksHalf() {
        lStickServo.setPosition(0.50);
        rStickServo.setPosition(0.60);
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
