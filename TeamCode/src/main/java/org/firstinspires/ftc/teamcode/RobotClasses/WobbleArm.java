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

    private int wobbleUpPos = Constants.WOBBLE_UP_POS;
    private int wobbleDownPos = Constants.WOBBLE_DOWN_POS;
    private int wobbleUpTeleopPos = Constants.WOBBLE_UP_TELEOP_POS;
    private int wobbleDownTeleopPos = Constants.WOBBLE_DOWN_TELEOP_POS;
    private double clampWobblePos = Constants.CLAMP_WOBBLE_POS;
    private double unClampWobblePos = Constants.UNCLAMP_WOBBLE_POS;

    public WobbleArm(LinearOpMode op, boolean isAuto) {
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");

        clampWobble();
        this.isAuto = isAuto;
        if (isAuto) {
            armUp();
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setPower(0);
        }

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void setPower(double power) {
        wobbleMotor.setPower(power);
    }

    public void armUp() {
        clampWobble();
        if (isAuto) {
            wobbleMotor.setTargetPosition(wobbleUpPos);
        } else {
            wobbleMotor.setTargetPosition(wobbleUpTeleopPos);
        }
        wobbleMotor.setPower(0.4);
    }

    public void armDown() {
        clampWobble();
        if (isAuto) {
            wobbleMotor.setTargetPosition(wobbleDownPos);
        } else {
            wobbleMotor.setTargetPosition(wobbleDownTeleopPos);
        }
        wobbleMotor.setPower(0.4);
    }

    public void setArmPosition(int position) {
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setPower(0.4);
    }

    public void clampWobble() {
        wobbleServo.setPosition(clampWobblePos);
    }

    public void unClampWobble() {
        wobbleServo.setPosition(unClampWobblePos);
    }
}