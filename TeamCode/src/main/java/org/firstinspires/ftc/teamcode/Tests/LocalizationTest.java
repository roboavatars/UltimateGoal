package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.T265;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name = "Combined Localization Test")
@Disabled
public class LocalizationTest extends LinearOpMode {
    private double odoX, odoY, odoTheta;
    private double odoVx, odoVy, odoW;
    private double camX, camY, camTheta;
    private double x, y, theta;
    private double startTime, runTime, curTime, prevTime, timeDiff;

    private double startX = 111;
    private double startY = 63;
    private double startTheta = PI/2;

    @Override
    public void runOpMode() {
        MecanumDrivetrain dt = new MecanumDrivetrain(this, startX, startY, startTheta);
        T265 t265 = new T265(this, startX, startY, startTheta);

        waitForStart();
        t265.startCam();
        startTime = System.currentTimeMillis();

        while(opModeIsActive()) {
            // Driving
            dt.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            // Update Time Variables
            curTime = (double) System.currentTimeMillis() / 1000;
            timeDiff = curTime - prevTime;
            runTime = curTime - startTime / 1000;
            prevTime = curTime;

            // Update Drivetrain Kinematics Variables
            dt.updatePose();

            odoVx = (dt.x - odoX) / timeDiff;
            odoVy = (dt.y - odoY) / timeDiff;
            odoW = (dt.theta - odoTheta) / timeDiff;

            odoX = dt.x;
            odoY = dt.y;
            odoTheta = dt.theta;

            // Update T265 Kinematics Variables
//            t265.sendOdometryData(odoVx, odoVy, odoTheta, odoW);
            t265.updateCamPose();

            camX = t265.getX();
            camY = t265.getY();
            camTheta = t265.getTheta();

            // Update Robot Kinematics Variables
            if (t265.confidence <= 1) {
                x = covariance(odoX, camX, 0);
                y = covariance(odoY, camY, 0);
                theta = covariance(odoTheta, camTheta, 0);
            } else if (t265.confidence == 2) {
                x = covariance(odoX, camX, 60);
                y = covariance(odoY, camY, 60);
                theta = covariance(odoTheta, camTheta, 60);
            } else if (t265.confidence == 3) {
                x = covariance(odoX, camX, runTime);
                y = covariance(odoY, camY, runTime);
                theta = covariance(odoTheta, camTheta, runTime);
            }

            // Dashboard
            drawField();
            drawDrivetrain(odoX, odoY, odoTheta, "blue");
            drawDrivetrain(camX, camY, camTheta, t265.confidenceColor());
            drawDrivetrain(x, y, theta, "black");
            addPacket("X", x);
            addPacket("Y", y);
            addPacket("Theta", theta);
            addPacket("Run Time", runTime);
            addPacket("Update Frequency (Hz)", 1 / timeDiff);
            sendPacket();
        }

        t265.stopCam();
    }

    private double covariance(double odo, double t265, double time) {
        if (time <= 5) {
            return odo;
        } else if (time >= 120) {
            return t265;
        } else {
            return odo * (1 - time / 120) + t265 * time / 120;
        }
    }
}