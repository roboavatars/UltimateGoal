package org.firstinspires.ftc.teamcode.Tests.T265Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.T265;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "0 T265 Test")
@Config
//@Disabled
public class T265Test extends LinearOpMode {

    public static double startX = 111;
    public static double startY = 63;
    public static double startTheta = Math.PI/2;
    private double x;
    private double y;
    private double theta;
    private double prevTime;

    @Override
    public void runOpMode() {
        MecanumDrivetrain dt = new MecanumDrivetrain(this, startX, startY, startTheta);
        T265 t265 = new T265(this, startX, startY, startTheta);

        waitForStart();
        t265.startCam();

        while(opModeIsActive()) {

            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            if (gamepad1.x) {
                t265.setCameraPose(startX, startY, startTheta);
            }
            
            if (gamepad1.a) {
                t265.exportMap();
            }

            if (gamepad1.b) {
                t265.stopCam();
            }

            t265.updateCamPose();
            x = t265.getX();
            y = t265.getY();
            theta = t265.getTheta();

            double curTime = (double) System.currentTimeMillis() / 1000;
            double timeDiff = curTime - prevTime;
            prevTime = curTime;

            drawField();
            drawRobot(x, y, theta, t265.confidenceColor());
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("isEmpty", t265.isEmpty);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            sendPacket();

            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("Theta: ", theta);
            telemetry.update();
        }

        t265.stopCam();
    }
}