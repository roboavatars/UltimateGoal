package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp()
public class AlignTest extends LinearOpMode {
    private TouchSensor limitSwitch;
    private boolean aligned = false;

    @Override
    public void runOpMode() {
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        waitForStart();

        while (opModeIsActive()) {
            aligned = limitSwitch.isPressed();

            addPacket("Aligned", aligned);
            sendPacket();
        }
    }
}