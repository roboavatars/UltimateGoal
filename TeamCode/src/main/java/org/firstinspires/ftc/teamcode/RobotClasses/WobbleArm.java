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
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        setPosition(Constants.WOBBLE_UP_POS);
        wobbleMotor.setPower(0.4);
    }

    public void armDown() {
        clampWobble();
        setPosition(Constants.WOBBLE_DOWN_POS);
        wobbleMotor.setPower(0.4);
    }

    private void setPosition(int position) {
        wobbleMotor.setTargetPosition(position);
    }

    public void setArmPosition(int position) {
        setPosition(position);
        wobbleMotor.setPower(0.4);
    }

    public void clampWobble() {
        wobbleServo.setPosition(Constants.CLAMP_WOBBLE_POS);
    }

    public void unClampWobble() {
        wobbleServo.setPosition(Constants.UNCLAMP_WOBBLE_POS);
    }
}