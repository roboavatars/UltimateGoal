package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotorEx intakeMotor;
    private DcMotorEx wobbleMotor;
    private Servo wobbleServo;

    private int wobbleHomePos = 0;
    private int wobbleUpPos = 0;
    private int wobbleClampPos = 0;
    private int wobbleReleasePos = 0;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        wobbleMotor = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");

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

    public void wobbleHome() {
        wobbleMotor.setTargetPosition(wobbleHomePos);
    }

    public void wobbleUp() {
        wobbleMotor.setTargetPosition(wobbleUpPos);
    }

    public void wobbleClamp() {
        wobbleServo.setPosition(wobbleClampPos);
    }

    public void wobbleRelease() {
        wobbleServo.setPosition(wobbleReleasePos);
    }
}
