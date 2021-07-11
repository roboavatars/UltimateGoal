package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.atan2;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@TeleOp(name = "Odometry Test")
public class OdoTest extends LinearOpMode {
    private boolean started;
    private double startTime;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 87, 81, PI/2, false);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.x) {
                robot.resetOdo(87, 81, PI/2);
            }

            if (started) {
                double t = (System.currentTimeMillis() - startTime) / 1000;
                robot.setTargetPoint(87 + 24 * sin(t) * cos(t), 81 + 48 * sin(t), atan2(2 * cos(t), cos(2*t)));
                if (t > 4*PI) {
                    started = false;
                }
            } else {
                robot.setTargetPoint(87, 81, PI/2);
            }

            if (gamepad1.a) {
                if (!started) {
                    started = true;
                    startTime = System.currentTimeMillis();
                }
            }

            addPacket("pod1", robot.drivetrain.pod1);
            addPacket("pod2", robot.drivetrain.pod2);
            addPacket("pod3", robot.drivetrain.pod3);
            robot.update();
        }
    }
}