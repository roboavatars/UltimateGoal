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
            wobbleMotor.setTargetPosition(Constants.WOBBLE_UP_POS);
        } else {
            wobbleMotor.setTargetPosition(Constants.WOBBLE_UP_TELEOP_POS);
        }
        wobbleMotor.setPower(0.4);
    }

    public void armDown() {
        clampWobble();
        if (isAuto) {
            wobbleMotor.setTargetPosition(Constants.WOBBLE_DOWN_POS);
        } else {
            wobbleMotor.setTargetPosition(Constants.WOBBLE_DOWN_TELEOP_POS);
        }
        wobbleMotor.setPower(0.4);
    }

    public void setArmPosition(int position) {
        wobbleMotor.setTargetPosition(position);
        wobbleMotor.setPower(0.4);
    }

    public void clampWobble() {
        wobbleServo.setPosition(Constants.CLAMP_WOBBLE_POS);
    }

    public void unClampWobble() {
        wobbleServo.setPosition(Constants.UNCLAMP_WOBBLE_POS);
    }
}