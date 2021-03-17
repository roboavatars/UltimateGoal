package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class WobbleArm {

    private Servo armServo;
    private Servo clampServo;

    public WobbleArm(LinearOpMode op) {
        armServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        clampServo = op.hardwareMap.get(Servo.class, "clampServo");

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
        setClampPosition(Constants.WOBBLE_CLAMP_POS);
    }

    public void unClamp() {
        setClampPosition(Constants.WOBBLE_UNCLAMP_POS);
    }

    public void setClampPosition(double position) {
        clampServo.setPosition(position);
    }
}