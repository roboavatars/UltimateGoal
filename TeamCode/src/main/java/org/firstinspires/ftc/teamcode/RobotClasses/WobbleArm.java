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
        leftClampServo = op.hardwareMap.get(Servo.class, "leftClamp");
        rightClampServo = op.hardwareMap.get(Servo.class, "rightClamp");

        armUp();

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void armInside() {
        setArmPosition(Constants.WOBBLE_INSIDE_POS);
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
        setClampPosition(Constants.WOBBLE_CLAMP_POS_LEFT, Constants.WOBBLE_CLAMP_POS_RIGHT);
    }

    public void unClamp() {
        setClampPosition(Constants.WOBBLE_UNCLAMP_POS_LEFT, Constants.WOBBLE_UNCLAMP_POS_RIGHT);
    }

    public void setClampPosition(double leftClampPosition, double rightClampPosition) {
        leftClampServo.setPosition(leftClampPosition);
        rightClampServo.setPosition(rightClampPosition);
    }
}