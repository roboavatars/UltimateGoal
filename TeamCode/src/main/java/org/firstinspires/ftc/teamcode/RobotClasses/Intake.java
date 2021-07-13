package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    public DcMotorEx intakeMotor;
    public DcMotorEx intakeMotor2;
    private Servo blockerServo;
    private Servo stackServo;
    private Servo bumperLeft;
    private Servo bumperRight;

    private double lastIntakePow = 0;
    private double lastBlocker = 0;

    public boolean on = false;
    public boolean reverse = false;
    public boolean forward = false;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor2 = op.hardwareMap.get(DcMotorEx.class, "intake2");

        blockerServo = op.hardwareMap.get(Servo.class, "blocker");
        stackServo = op.hardwareMap.get(Servo.class, "stackServo");

        bumperLeft = op.hardwareMap.get(Servo.class, "bumperLeft");
        bumperRight = op.hardwareMap.get(Servo.class, "bumperRight");

//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!isAuto) {
            blockerDown();
        } else {
            blockerHome();
        }
        stackHome();

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void on() {
        setPower(1);
    }

    public void verticalOn() {
        intakeMotor.setPower(0);
        intakeMotor2.setPower(1);
        lastIntakePow = 2;
    }

    public void reverse() {
        setPower(-1);
    }

    public void off() {
        setPower(0);
    }

    public void setPower(double power) {
        if (power != lastIntakePow) {
            intakeMotor.setPower(power);
            intakeMotor2.setPower(power);

            on = power != 0;
            forward = power > 0;
            reverse = power < 0;
            lastIntakePow = power;
        }
    }

    public void blockerHome() {
        setBlocker(Constants.BLOCKER_HOME_POS);
    }

    public void blockerUp() {
        setBlocker(Constants.BLOCKER_UP_POS);
    }

    public void blockerDown() {
        setBlocker(Constants.BLOCKER_DOWN_POS);
    }

    public void setBlocker(double position) {
        if (position != lastBlocker) {
            blockerServo.setPosition(position);
            lastBlocker = position;
        }
    }

    public void stackHome() {
        stackServo.setPosition(Constants.STACK_HOME_POS);
    }

    public void stackOut() {
        stackServo.setPosition(Constants.STACK_OUT_POS);
    }

//    public void stackDown() {
//        stackServo.setPosition(0.98);
//    }

    public void autoBumpers() {}
}
