package org.firstinspires.ftc.teamcode.Tests.T265Tests;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

@TeleOp(name="Basic T265 Test")
public class DefaultT265Test extends LinearOpMode {

    private static T265Camera slamra = null;

    public static double startX = 111;
    public static double startY = 63;
    public static double startTheta = Math.PI/2;
    private static final double INCH_TO_METER = 0.0254;

    @Override
    public void runOpMode() {
        MecanumDrivetrain dt = new MecanumDrivetrain(this, startX, startY, startTheta);
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        waitForStart();

        slamra.start();
        slamra.setPose(new Pose2d(startX * INCH_TO_METER, startY * INCH_TO_METER, new Rotation2d(startTheta)));

        while(opModeIsActive()) {

            dt.setControls(-0.8 * gamepad1.left_stick_y, -0.8 * gamepad1.left_stick_x, -gamepad1.right_stick_x);

            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up == null) continue;

            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            drawField();
            drawRobot(translation.getX(), translation.getY(), rotation.getRadians(), "blue");

            addPacket("X", translation.getX());
            addPacket("Y", translation.getY());
            addPacket("Theta", rotation.getRadians());
            sendPacket();
        }

        slamra.stop();
    }

}