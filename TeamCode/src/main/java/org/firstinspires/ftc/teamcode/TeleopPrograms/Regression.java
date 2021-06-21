package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

@TeleOp @Config
public class Regression extends LinearOpMode {

    public int startX = 111;
    public int startY = 63;
    public double startTheta = PI/2;

    private final double xyTol = 1;
    private final double thetaTol = PI/35;

    private Robot robot;
    public boolean flywheelToggle = false, flywheelOn = false;
    public boolean magToggle = false, magUp = false;

    public static double flywheelVelocity = Constants.HIGH_GOAL_BACK_VELOCITY;
    public static double flapPos = Constants.FLAP_BACK_POS;
    public static double theta0 = 1.875;
    public static double theta1 = 1.810;
    public static double theta2 = 1.725;

    @Override
    public void runOpMode() {

        robot = new Robot(this, startX, startY, startTheta, false);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_trigger > 0) {
                robot.intake.on();
            } else if (gamepad1.right_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            robot.shooter.setFlapPosition(flapPos);

            if (gamepad1.left_bumper && !flywheelToggle) {
                flywheelToggle = true;
                if (flywheelOn) {
                    robot.shooter.magHome();
                    robot.shooter.flywheelOff();
                } else {
                    robot.shooter.magShoot();
                    robot.shooter.setFlywheelVelocity(flywheelVelocity);
                }
                flywheelOn = !flywheelOn;
            } else if (!gamepad1.left_bumper && flywheelToggle) {
                flywheelToggle = false;
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedTop();
            } else {
                robot.shooter.feedHome();
            }

            if (gamepad2.b) {
                robot.intake.blockerUp();
            } else {
                robot.intake.blockerDown();
            }

            if (gamepad1.a && !magToggle) {
                magToggle = true;
                if (magUp) {
                    robot.shooter.magHome();
                } else {
                    robot.shooter.magShoot();
                }
                magUp = !magUp;
            } else if (!gamepad1.a && magToggle) {
                magToggle = false;
            }

            if (gamepad1.x) {
                robot.resetOdo(111, 63, Math.PI/2);
            }

//            if (gamepad1.dpad_up) {
//                robot.shooter.setFlapPosition(robot.shooter.getFlapPosition() + 0.001);
//            } else if (gamepad1.dpad_down) {
//                robot.shooter.setFlapPosition(robot.shooter.getFlapPosition() - 0.001);
//            }

            if (gamepad1.dpad_left) {
                robot.setTargetPoint(111, 63, theta0);
            } else if (gamepad1.dpad_up) {
                robot.setTargetPoint(111, 63, theta1);
            } else if (gamepad1.dpad_right) {
                robot.setTargetPoint(111, 63, theta2);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -gamepad1.right_stick_x);
            }

//            if (gamepad1.right_trigger > 0) {
//                double[] target = robot.shootTargets(robot.psShootPos[0], robot.psShootPos[1], PI / 2, 2);
//
//                if (!(Math.abs(robot.x - target[0]) < xyTol && Math.abs(robot.y - target[1]) < xyTol && Math.abs(robot.theta - target[2]) < thetaTol)) {
//                    robot.setTargetPoint(target[0], target[1], target[2]);
//                }
//            }

//            robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            robot.update();

            Robot.log(Robot.round(robot.x) + ", " + Robot.round(robot.y) + ", " + Robot.round(robot.theta));

            telemetry.addData("Robot X", robot.x);
            telemetry.addData("Robot Y", robot.y);
            telemetry.addData("Robot Theta", robot.theta);
            telemetry.addData("Shooter Velocity", robot.shooter.getFlywheelVelocity());
            telemetry.update();
        }
    }
}