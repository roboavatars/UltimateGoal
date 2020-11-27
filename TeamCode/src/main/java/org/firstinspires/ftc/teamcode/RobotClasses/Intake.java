package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    private DcMotorEx intakeMotor;
    private Servo lStickServo;
    private Servo rStickServo;

    private final double lHomePos = 0.92;
    private final double rHomePos = 0;
    private final double lOutPos = 0;
    private final double rOutPos = 0.85;
    private final int wobbleUpPos = -80;
    private final int wobbleDownPos = -800;
    private final double wobbleClampPos = 0.20;
    private final double wobbleReleasePos = 0.75;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        lStickServo = op.hardwareMap.get(Servo.class, "leftStick");
        rStickServo = op.hardwareMap.get(Servo.class, "rightStick");

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void intakeOn() {
        intakeMotor.setPower(-1);
    }

    public void intakeRev() {
        intakeMotor.setPower(1);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void setIntakePow(double power) {
        intakeMotor.setPower(power);
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
