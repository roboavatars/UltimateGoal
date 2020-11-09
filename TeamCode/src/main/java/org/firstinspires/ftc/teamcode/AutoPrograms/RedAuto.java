package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.StackHeightDetector;
import org.firstinspires.ftc.teamcode.OpenCV.StackHeightPipeline.RingCase;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Point2D;

@Autonomous
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        final double PI = Math.PI;

        Robot robot = new Robot(this, 100, 20, PI/2);
        robot.logger.startLogging();

        StackHeightDetector detector = new StackHeightDetector(this);
        detector.start();

        boolean getStartStack = false;
        boolean deliverWobble = false;
        boolean wobbleTwo = false;
        boolean park = false;

        double getStartStackTime;
        double deliverWobbleTime;
        double wobbleTwoTime;
        double parkTime;

        waitForStart();

        RingCase ringCase = detector.getResult();
        Point2D[] wobbleDelivery = {
                new Point2D(130, 75), new Point2D(105, 100), new Point2D(125, 120)
        };

        Point2D wobbleCor;
        if (ringCase == RingCase.Four) {
            wobbleCor = wobbleDelivery[2];
            /*deliverWobbleTime =
            wobbleTwoTime = */
        } else if (ringCase == RingCase.One) {
            wobbleCor = wobbleDelivery[1];
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        } else {
            wobbleCor = wobbleDelivery[0];
            /*deliverWobbleTime =
            wobbleTwoTime =*/
        }

        detector.stop();

        Path getStartStackPath = null;
        Path deliverWobblePath = null;
        Path wobbleTwoPath = null;
        Path parkPath = null;

        ElapsedTime time = new ElapsedTime();

        while(opModeIsActive()) {

            if (!getStartStack) {

            } else if (!deliverWobble) {

            } else if (!wobbleTwo) {

            } else if (!park) {

            }

            else {
                robot.drivetrain.setControls(0,0,0);
            }

            robot.update();
        }

        robot.update();
        robot.logger.stopLogging();
    }
}
