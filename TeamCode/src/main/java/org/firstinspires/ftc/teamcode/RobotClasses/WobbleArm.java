package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class WobbleArm {

    private DcMotorEx wobbleMotor;
    private Servo wobbleServo;
    private boolean isAuto;

    private final int wobbleUpPos = -80;
    private final int wobbleDownPos = -830;
    private final int wobbleUpTeleopPos = -80;
    private final int wobbleDownTeleopPos = -1000;
    private final double wobbleClampPos = 0.20;
    private final double wobbleReleasePos = 0.75;

    public WobbleArm(LinearOpMode op, boolean isAuto) {
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleUp();
        wobbleClamp();
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.isAuto = isAuto;

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void wobbleUp() {
        wobbleClamp();
        if (isAuto) {
            wobbleMotor.setTargetPosition(wobbleUpPos);
        } else {
            wobbleMotor.setTargetPosition(wobbleUpTeleopPos);
        }
        wobbleMotor.setPower(0.4);
    }

    public void wobbleDown() {
        wobbleClamp();
        if (isAuto) {
            wobbleMotor.setTargetPosition(wobbleDownPos);
        } else {
            wobbleMotor.setTargetPosition(wobbleDownTeleopPos);
        }
        wobbleMotor.setPower(0.4);
    }

    public void setWobbleMotorPosition(int position) {
        wobbleMotor.setTargetPosition(position);
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