package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class WobbleArm {
    private Servo armServo;
    private Servo leftClampServo;
    private Servo rightClampServo;

    public boolean armUp = false;
    public boolean armDown = false;
    public boolean armIn = false;
    public boolean clamped = false;
    public boolean unClamped = false;
    public boolean clamIn = false;

    public WobbleArm(LinearOpMode op, boolean isAuto) {
        armServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        leftClampServo = op.hardwareMap.get(Servo.class, "leftClamp");
        rightClampServo = op.hardwareMap.get(Servo.class, "rightClamp");

        if (isAuto) {
            setArmPosition(0.8);
            clamp();
        } else {
            armInside();
            clawIn();
        }

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void armInside() {
        setArmPosition(Constants.WOBBLE_INSIDE_POS);
        armUp = false;
        armDown = false;
        armIn = true;
    }

    public void armUp() {
        setArmPosition(Constants.WOBBLE_UP_POS);
        armUp = true;
        armDown = false;
        armIn = false;
    }

    public void armDown() {
        setArmPosition(Constants.WOBBLE_DOWN_POS);
        armUp = false;
        armDown = true;
        armIn = false;
    }

    public void setArmPosition(double position) {
        armServo.setPosition(position);
    }

    public void clamp() {
        setClampPosition(Constants.WOBBLE_CLAMP_POS_LEFT, Constants.WOBBLE_CLAMP_POS_RIGHT);
        clamped = true;
        unClamped = false;
        clamIn = false;
    }

    public void unClamp() {
        setClampPosition(Constants.WOBBLE_UNCLAMP_POS_LEFT, Constants.WOBBLE_UNCLAMP_POS_RIGHT);
        clamped = false;
        unClamped = true;
        clamIn = false;
    }

    public void clawIn() {
        setClampPosition(Constants.WOBBLE_HOME_POS_LEFT, Constants.WOBBLE_HOME_POS_RIGHT);
        clamped = false;
        unClamped = false;
        clamIn = true;
    }

    public void setClampPosition(double leftClampPosition, double rightClampPosition) {
        leftClampServo.setPosition(leftClampPosition);
        rightClampServo.setPosition(rightClampPosition);
    }
}