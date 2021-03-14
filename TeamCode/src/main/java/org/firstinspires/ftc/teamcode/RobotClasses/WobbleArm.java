package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class WobbleArm {

    private Servo wobbleServo;

    public WobbleArm(LinearOpMode op) {
        wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");
        armUp();

        op.telemetry.addData("Status", "Wobble Arm initialized");
    }

    public void armUp() {
        setPosition(Constants.WOBBLE_UP_POS);
    }

    public void armDown() {
        setPosition(Constants.WOBBLE_DOWN_POS);
    }

    public void setPosition(double position) {
        wobbleServo.setPosition(position);
    }
}