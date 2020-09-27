package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeIndex {

    private DcMotorEx intakeMotor;
    private DcMotorEx indexMotor;
    private DistanceSensor ringSensor;

    public IntakeIndex(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        indexMotor = op.hardwareMap.get(DcMotorEx.class, "index");
        ringSensor = op.hardwareMap.get(DistanceSensor.class, "ringSensor");

        op.telemetry.addData("Status", "Intake and Indexer initialized");
    }

    public void intakeOn() {
        intakeMotor.setPower(1);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void indexRings() {
        indexMotor.setPower(1);
    }

    public void stopIndex() {
        indexMotor.setPower(0);
    }

    public double getDistance() {
        return ringSensor.getDistance(DistanceUnit.INCH);
    }

}
