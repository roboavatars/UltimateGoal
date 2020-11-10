package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

    private DcMotorEx intakeMotor;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intakeMotor");

        setPower(0);

        op.telemetry.addData("Status", "Intake initialized");
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

}
