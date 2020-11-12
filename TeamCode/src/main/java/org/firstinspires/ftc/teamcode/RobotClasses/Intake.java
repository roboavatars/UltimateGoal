package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotorEx intakeMotor;
//    private Servo wobbleServo;

//    private double wobbleHomePos = 0;
//    private double wobbleUpPos = 0;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        //wobbleServo = op.hardwareMap.get(Servo.class, "wobbleServo");

//        wobbleHome();

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void intakeOn() {
        intakeMotor.setPower(-1);
    }

    public void intakeRev() {
        intakeMotor.setPower(1);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

//    public void wobbleHome() {
//        wobbleServo.setPosition(wobbleHomePos);
//    }
//
//    public void wobbleUp() {
//        wobbleServo.setPosition(wobbleUpPos);
//    }

}
