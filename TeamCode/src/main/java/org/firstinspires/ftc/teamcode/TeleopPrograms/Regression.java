package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.PI;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
public class Regression extends LinearOpMode {

    public int startX = 111;
    public int startY = 63;
    public double startTheta = PI/2;

    public int v = 1560;
    public double blocker = 0.31;

    private final double xyTol = 1;
    private final double thetaTol = PI/35;

    private Robot robot;
    private final boolean robotCentric = false;
    public boolean flywheelToggle = false;
    public boolean flywheelOn = false;

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

            if (gamepad1.left_bumper && !flywheelToggle) {
                flywheelToggle = true;
                if (flywheelOn) {
                    robot.shooter.magHome();
                    robot.shooter.flywheelOff();
                } else {
                    robot.shooter.magShoot();
                    robot.shooter.setVelocity(v);
                }
                flywheelOn = !flywheelOn;
            } else if (!gamepad1.left_bumper && flywheelToggle) {
                flywheelToggle = false;
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedHome();
            } else {
                robot.shooter.feedTop();
            }

            if (gamepad2.a) {
                robot.intake.setBlocker(blocker);
            } else {
                robot.intake.blockerDown();
            }

            if (gamepad1.x) {
                robot.resetOdo(111, 63, Math.PI/2);
            }

//            if (gamepad1.right_trigger > 0) {
//                double[] target = robot.shootTargets(robot.psShootPos[0], robot.psShootPos[1], PI / 2, 2);
//
//                if (!(Math.abs(robot.x - target[0]) < xyTol && Math.abs(robot.y - target[1]) < xyTol && Math.abs(robot.theta - target[2]) < thetaTol)) {
//                    robot.setTargetPoint(target[0], target[1], target[2]);
//                }
//            }

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            if (gamepad1.dpad_up) {
                v += 5;
                robot.intake.setBlocker(robot.intake.getBlockerPos() + 0.001);
                blocker += 0.001;
            }
            if (gamepad1.dpad_down) {
                v -= 5;
                robot.intake.setBlocker(robot.intake.getBlockerPos() - 0.001);
                blocker -= 0.001;
            }

            robot.update();

            Robot.log(Robot.round(robot.x) + ", " + Robot.round(robot.y) + ", " + Robot.round(robot.theta));

            telemetry.addData("Robot X", robot.x);
            telemetry.addData("Robot Y", robot.y);
            telemetry.addData("Robot Theta", robot.theta);
            telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
            telemetry.addData("vel", v);
            telemetry.update();

            /*
            0- 114.32136, 62.49184, 1.9383
            1- 114.56906, 62.28271, 1.82575
            2- 114.50752, 62.77721, 1.77424
             */

        }
    }
}