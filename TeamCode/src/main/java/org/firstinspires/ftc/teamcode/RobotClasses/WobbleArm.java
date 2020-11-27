package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class WobbleArm {

    private DcMotorEx wobbleMotor;
    private Servo wobbleServo;

    private final int wobbleUpPos = -80;
    private final int wobbleDownPos = -800;
    private final double wobbleClampPos = 0.20;
    private final double wobbleReleasePos = 0.75;

    public WobbleArm(LinearOpMode op) {
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleUp();
        wobbleClamp();
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void wobbleUp() {
        wobbleClamp();
        wobbleMotor.setTargetPosition(wobbleUpPos);
        wobbleMotor.setPower(0.4);
    }

    public void wobbleDown() {
        wobbleClamp();
        wobbleMotor.setTargetPosition(wobbleDownPos);
        wobbleMotor.setPower(0.4);
    }

    public void wobbleHome() {
        wobbleClamp();
        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setPower(0.4);
    }

    public void wobbleClamp() {
        wobbleServo.setPosition(wobbleClampPos);
    }

    public void wobbleRelease() {
        wobbleServo.setPosition(wobbleReleasePos);
    }
}