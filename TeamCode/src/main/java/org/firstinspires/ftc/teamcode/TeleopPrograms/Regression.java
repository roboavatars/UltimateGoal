package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@TeleOp(name = "1 Teleop")
@Config
public class Regression extends LinearOpMode {

    public int startX = 111;
    public int startY = 63;
    public double startTheta = PI/2;

    private Robot robot;
    private boolean flywheelToggle = false, flywheelOn = false;
    private boolean magToggle = false, magUp = false;
    private boolean downToggle = false, armDown = false;
    private boolean clampToggle = false, clamped = false;

    public boolean started = false;
    public double flickTime;
    public int numRings = 0;
    public static int delay = 125;

    public static double intakePow = 1;
    public static double transferPow = 0.5;

    public static double flywheelVelocity = 1620;
    public static double flapPos = Constants.FLAP_BACK_POS;
    public static double theta0 = 1.875;
    public static double theta1 = 1.810;
    public static double theta2 = 1.725;

    @Override
    public void runOpMode() {
        robot = new Robot(this, startX, startY, startTheta, false);

        robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0) {
                robot.intake.setPower(intakePow, transferPow);
            } else if (gamepad1.right_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

//            if (gamepad1.left_bumper && !flywheelToggle) {
//                flywheelToggle = true;
//                if (flywheelOn) {
//                    robot.shooter.magHome();
//                    robot.shooter.flywheelOff();
//                } else {
//                    robot.shooter.magShoot();
//                    robot.shooter.setFlywheelVelocity(flywheelVelocity);
//                }
//                flywheelOn = !flywheelOn;
//            } else if (!gamepad1.left_bumper && flywheelToggle) {
//                flywheelToggle = false;
//            }
//
//            if (flywheelOn) {
//                robot.shooter.setFlywheelVelocity(flywheelVelocity);
//            }
//
//            if (gamepad1.a && !magToggle) {
//                magToggle = true;
//                if (magUp) {
//                    robot.shooter.magHome();
//                } else {
//                    robot.shooter.magShoot();
//                }
//                magUp = !magUp;
//            } else if (!gamepad1.a && magToggle) {
//                magToggle = false;
//            }

            if (gamepad1.right_bumper) {
                robot.highGoalShoot();
//                if (!started) {
//                    started = true;
//                    flickTime = System.currentTimeMillis();
//                    numRings = 3;
//                }
            }

//            if (started && System.currentTimeMillis() - flickTime > delay && numRings > 0) {
//                if (robot.shooter.feedHome) {
//                    robot.shooter.feedShoot();
//                } else {
//                    robot.shooter.feedHome();
//                    numRings--;
//                }
//                flickTime = System.currentTimeMillis();
//
//                if (numRings == 0) {
//                    started = false;
//                }
//            }

            if (gamepad1.y) {
                robot.intake.blockerUp();
            } else {
                robot.intake.blockerDown();
            }

            if (gamepad1.x) {
                robot.resetOdo(111, 63, Math.PI/2);
            }

            robot.intake.autoBumpers(robot.x, robot.y, robot.theta, 12);


            if (gamepad2.dpad_down && !downToggle) {
                downToggle = true;
                if (armDown) {
                    if (gamepad2.left_trigger > 0){
                        robot.intake.bumpersHome();
                    }
                    robot.wobbleArm.armUp();
                } else {
                    robot.wobbleArm.armDown();
                    robot.intake.bumpersOut();
                }
                armDown = !armDown;
            } else if (!gamepad2.dpad_down && downToggle) {
                downToggle = false;
            }

            if (gamepad2.dpad_left && !clampToggle) {
                clampToggle = true;
                if (clamped) {
                    robot.wobbleArm.unClamp();
                } else {
                    robot.wobbleArm.clamp();
                }
                clamped = !clamped;
            } else if (!gamepad2.dpad_left && clampToggle) {
                clampToggle = false;
            }

//            if (gamepad1.dpad_left) {
//                robot.setTargetPoint(111, 63, theta0);
//            } else if (gamepad1.dpad_up) {
//                robot.setTargetPoint(111, 63, theta1);
//            } else if (gamepad1.dpad_right) {
//                robot.setTargetPoint(111, 63, theta2);
//            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -gamepad1.right_stick_x);
//            }

            addPacket("d", Math.sqrt(Math.pow(robot.x - 108, 2) + Math.pow(144 - robot.y, 2)));
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