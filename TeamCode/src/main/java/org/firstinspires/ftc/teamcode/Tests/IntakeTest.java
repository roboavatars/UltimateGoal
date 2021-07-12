package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Intake Test")
@Config
//@Disabled
public class IntakeTest extends LinearOpMode {
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;
    private Servo blockerServo;
    private Servo stackServo;

    public static double intakePower;
    public static double intake2Power;
    public static double blockerPos;
    public static double stackServoPos;

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake2");

        blockerServo = hardwareMap.get(Servo.class, "blocker");
        stackServo = hardwareMap.get(Servo.class, "stackServo");

        waitForStart();

        while (opModeIsActive()) {
            intakeMotor.setPower(intakePower);
            intakeMotor2.setPower(intake2Power);
            blockerServo.setPosition(blockerPos);
            stackServo.setPosition(stackServoPos);

            addPacket("intakePower", intakePower);
            addPacket("intake2Power", intake2Power);
            addPacket("blockerPos", blockerPos);
            addPacket("stackServoPos", stackServoPos);
            sendPacket();
        }
    }
}
