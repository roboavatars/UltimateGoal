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

    private final double lHomePos = 1;
    private final double lOutPos = 0.23;
    private final double rHomePos = 0.25;
    private final double rOutPos = 0.9;

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        lStickServo.setPosition(lHomePos);
        rStickServo.setPosition(rHomePos);
    }

    public void sticksHomeAuto() {
        lStickServo.setPosition(0.55);
        rStickServo.setPosition(rHomePos);
    }

    public void rStickDown() {
        rStickServo.setPosition(1.0);
    }

    public void sticksHalf() {
        lStickServo.setPosition(0.50);
        rStickServo.setPosition(0.60);
    }

    public void sticksOut() {
        lStickServo.setPosition(lOutPos);
        rStickServo.setPosition(rOutPos);
    }

    public void stickLeft(double position) {
        lStickServo.setPosition(position);
    }

    public void stickRight(double position) {
        rStickServo.setPosition(position);
    }
}
