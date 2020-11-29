package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    private DcMotorEx intakeMotor;
    private Servo lStickServo;
    private Servo rStickServo;
    private CRServo intakeServo;

    private final double lHomePos = 0.92;
    private final double rHomePos = 0;
    private final double lOutPos = 0;
    private final double rOutPos = 0.85;

    public boolean intakeOn = false;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");
        intakeServo = op.hardwareMap.get(CRServo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void intakeOn() {
        intakeMotor.setPower(-1);
        intakeServo.setPower(-1);
        intakeOn = true;
    }

    public void intakeRev() {
        intakeMotor.setPower(1);
        intakeServo.setPower(1);
        intakeOn = true;
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
        intakeOn = false;
    }

    public void setIntakePow(double power) {
        intakeMotor.setPower(power);
        intakeServo.setPower(power);
        intakeOn = power != 0;
    }

    public void rStickDown() {
        rStickServo.setPosition(0.9);
    }

    public void sticksHalf() {
        lStickServo.setPosition(0.5);
        rStickServo.setPosition(0.5);
    }

    public void sticksHome() {
        lStickServo.setPosition(lHomePos);
        rStickServo.setPosition(rHomePos);
    }

    public void sticksOut() {
        lStickServo.setPosition(lOutPos);
        rStickServo.setPosition(rOutPos);
    }
}
