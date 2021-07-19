package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;

@TeleOp(name = "1 Teleop")
@Config
public class Regression extends LinearOpMode {

    public int startX = 111;
    public int startY = 63;
    public double startTheta = PI/2;

    private Robot robot;
    public boolean flywheelToggle = false, flywheelOn = false;
    public boolean magToggle = false, magUp = false;

    public boolean started = false;
    public double flickTime;
    public int numRings = 0;
    public static int delay = 125;

    public static double intakePow = 1;
    public static double transferPow = 0.3;

    public static double flywheelVelocity = 1620;
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
                robot.intake.setPower(intakePow, transferPow);
            } else if (gamepad1.right_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            if (gamepad1.left_bumper && !flywheelToggle) {
                flywheelToggle = true;
                if (flywheelOn) {
//                    robot.shooter.magHome();
                    robot.shooter.flywheelOff();
                } else {
//                    robot.shooter.magShoot();
                    robot.shooter.setFlywheelVelocity(flywheelVelocity);
                }
                flywheelOn = !flywheelOn;
            } else if (!gamepad1.left_bumper && flywheelToggle) {
                flywheelToggle = false;
            }

            if (flywheelOn) {
                robot.shooter.setFlywheelVelocity(flywheelVelocity);
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedShoot();
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
//            robot.shooter.magShoot();

            if (gamepad1.x) {
                robot.resetOdo(111, 63, Math.PI/2);
            }

            if (gamepad1.a) {
                if (!started) {
                    started = true;
                    flickTime = System.currentTimeMillis();
                    numRings = 3;
                }
            }

            if (started && System.currentTimeMillis() - flickTime > delay && numRings > 0) {
                if (robot.shooter.feedHome) {
                    robot.shooter.feedShoot();
                } else {
                    robot.shooter.feedHome();
                    numRings--;
                }
                flickTime = System.currentTimeMillis();

                if (numRings == 0) {
                    started = false;
                }
            }

            robot.shooter.setFlapPosition(flapPos);
//            if (gamepad1.dpad_up) {
//                robot.shooter.setFlapPosition(robot.shooter.getFlapPosition() + 0.001);
//            } else if (gamepad1.dpad_down) {
//                robot.shooter.setFlapPosition(robot.shooter.getFlapPosition() - 0.001);
//            }

            robot.intake.autoBumpers(robot.x, robot.y, robot.theta, 12);

            if (gamepad1.dpad_left) {
                robot.setTargetPoint(111, 63, theta0);
            } else if (gamepad1.dpad_up) {
                robot.setTargetPoint(111, 63, theta1);
            } else if (gamepad1.dpad_right) {
                robot.setTargetPoint(111, 63, theta2);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -gamepad1.right_stick_x);
            }

            robot.update();

            Robot.log(Robot.round(robot.x) + ", " + Robot.round(robot.y) + ", " + Robot.round(robot.theta));

            telemetry.addData("Robot X", robot.x);
            telemetry.addData("Robot Y", robot.y);
            telemetry.addData("Robot Theta", robot.theta);
            telemetry.addData("Turret Theta", robot.turretGlobalTheta);
            telemetry.addData("Shooter Velocity", robot.shooter.getFlywheelVelocity());
            telemetry.update();
        }
    }
}