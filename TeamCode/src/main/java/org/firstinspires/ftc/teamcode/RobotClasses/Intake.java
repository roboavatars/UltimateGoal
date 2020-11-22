package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotorEx intakeMotor;
    private DcMotorEx wobbleMotor;
    private Servo wobbleServo;

    private int wobbleUpPos = -100;
    private int wobbleDownPos = -750;
    private double wobbleClampPos = 0.3;
    private double wobbleReleasePos = 1;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void wobbleUp() {
        wobbleMotor.setTargetPosition(wobbleUpPos);
    }

    public void wobbleDown() {
        wobbleMotor.setTargetPosition(wobbleDownPos);
    }

    public void wobbleClamp() {
        wobbleServo.setPosition(wobbleClampPos);
    }

    public void wobbleRelease() {
        wobbleServo.setPosition(wobbleReleasePos);
    }
}
