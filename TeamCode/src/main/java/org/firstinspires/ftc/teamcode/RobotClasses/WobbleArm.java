package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class WobbleArm {

    private Servo armServo;
    private Servo leftClampServo;
    private Servo rightClampServo;


    public WobbleArm(LinearOpMode op) {
        armServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        leftClampServo = op.hardwareMap.get(Servo.class, "clampServo");
        rightClampServo = op.hardwareMap.get(Servo.class, "clampServo");

        armUp();
        clamp();

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void armUp() {
        setArmPosition(Constants.WOBBLE_UP_POS);
    }

    public void armDown() {
        setArmPosition(Constants.WOBBLE_DOWN_POS);
    }

    public void setArmPosition(double position) {
        armServo.setPosition(position);
    }

    public void clamp() {
        setClampPosition(Constants.WOBBLE_CLAMP_POS_LEFT_SERVO, Constants.WOBBLE_CLAMP_POS_RIGHT_SERVO);
    }

    public void unClamp() {
        setClampPosition(Constants.WOBBLE_UNCLAMP_POS_LEFT_SERVO, Constants.WOBBLE_UNCLAMP_POS_RIGHT_SERVO);
    }

    public void setClampPosition(double leftClampPosition, double rightClampPosition) {
        leftClampServo.setPosition(leftClampPosition);
        rightClampServo.setPosition(rightClampPosition);
    }
}