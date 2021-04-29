package org.firstinspires.ftc.teamcode.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp
@Config
public class MotionReplayer extends LinearOpMode {

    private Robot robot;
    private ArrayList<double[]> data;
    private int counter = 0;
    private double curTime = 0;
    public static double fileNumber = Logger.getLastFileNumber();

    @Override
    public void runOpMode() {
        robot = new Robot(this, 9, 111, 0, false);

        waitForStart();
        try {
            data = robot.logger.replay("/sdcard/FIRST/robotLogs/RobotData" + fileNumber + ".csv");
        } catch (Exception e) {
            data = new ArrayList<>();
        }

        while (opModeIsActive()) {
            if (data.size() > 0) {
                curTime = data.get(counter)[0];
                drawRobot(data.get(counter)[1], data.get(counter)[2], data.get(counter)[3]);
                sendPacket();
                sleep((long) curTime - (long) data.get(Math.max(0,counter-1))[0]);
                counter = Math.min(counter+1, data.size()-1);
            } else {
                break;
            }
        }
    }
}