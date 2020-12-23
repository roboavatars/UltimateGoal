package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Random;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp @Config
public class adbTest extends LinearOpMode {

    public static String str = "Hello :-)";

    @Override
    public void runOpMode() {

        packet.addLine("Im Alive");
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();

        waitForStart();

        while (opModeIsActive()) {
            packet.put("message", str);
            int data = new Random().nextInt(1000);
            packet.put("data", data);
            Robot.log("Data: " + data);
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
    }
}
