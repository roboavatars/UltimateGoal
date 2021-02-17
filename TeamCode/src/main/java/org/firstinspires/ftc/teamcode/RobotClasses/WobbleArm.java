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

    private double lastWobblePow = 0;

    public WobbleArm(LinearOpMode op, boolean isAuto) {
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        wobbleMotor = op.hardwareMap.get(DcMotorEx.class, "wobbleMotor");

        clamp();
        this.isAuto = isAuto;
        if (isAuto) {
            up();
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            wobbleMotor.setPower(0);
        }

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void setPower(double power) {
        if (power != lastWobblePow) {
            wobbleMotor.setPower(power);
            lastWobblePow = power;
        }
    }

    public void up() {
        clamp();
        armUp();
    }

    public void armUp() {
        setPosition(Constants.WOBBLE_UP_POS);
        wobbleMotor.setPower(0.4);
    }

    public void down() {
        clamp();
        armDown();
    }

    public void armDown() {
        setPosition(Constants.WOBBLE_DOWN_POS);
        wobbleMotor.setPower(0.4);
    }

    private void setPosition(int position) {
        wobbleMotor.setTargetPosition(position);
        Robot.log("wobble arm target pos: " + position);
    }

    public void setArmPosition(int position) {
        setPosition(position);
        wobbleMotor.setPower(0.4);
    }

    public int getPosition() {
        return wobbleMotor.getCurrentPosition();
    }

    public void clamp() {
        wobbleServo.setPosition(Constants.CLAMP_WOBBLE_POS);
    }

    public void unClamp() {
        wobbleServo.setPosition(Constants.UNCLAMP_WOBBLE_POS);
    }
}