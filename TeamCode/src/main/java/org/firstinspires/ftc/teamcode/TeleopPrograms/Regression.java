package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

@TeleOp(name = "2 Regression")
@Config
public class Regression extends LinearOpMode {

    public int startX = 87;
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
    public static int delay = 300;

    public static double intakePow = 1;
    public static double transferPow = 0.5;

    public static double flywheelVelocity = 1450;
    public static double theta0 = 1.671;
    public static double theta1 = 1.600;
    public static double theta2 = 1.498;

    @Override
    public void runOpMode() {
        robot = new Robot(this, startX, startY, startTheta, false);
        robot.logger.startLogging(false, true);

        robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

        waitForStart();

        robot.drivetrain.updateThetaError();

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

            if (flywheelOn) {
                robot.shooter.setFlywheelVelocity(flywheelVelocity);
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

            if (gamepad1.right_bumper) {
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

            if (gamepad1.y) {
                robot.intake.blockerHome();
            } else {
                robot.intake.blockerVert();
            }

            if (gamepad1.x) {
                robot.resetOdo(87, 63, Math.PI/2);
            }

            if (gamepad2.dpad_down && !downToggle) {
                downToggle = true;
                if (armDown) {
                    robot.wobbleArm.armUp();
                } else {
                    robot.wobbleArm.armDown();
                }
                armDown = !armDown;
            } else if (!gamepad2.dpad_down && downToggle) {
                downToggle = false;
            }

            if (!armDown) {
                robot.intake.autoBumpers(robot.x, robot.y, robot.theta, 12);
            } else {
                robot.intake.bumpersOut();
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

            if (gamepad1.dpad_left) {
                robot.thetaOffset -= 0.005;
            } else if (gamepad1.dpad_right) {
                robot.thetaOffset += 0.005;
            }

            if (gamepad1.dpad_left) {
                robot.setLockMode(Robot.TurretMode.PS_L);
            } else if (gamepad1.dpad_up) {
                robot.setLockMode(Robot.TurretMode.PS_C);
            } else if (gamepad1.dpad_right) {
                robot.setLockMode(Robot.TurretMode.PS_R);
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -gamepad1.right_stick_x);
            }

            addPacket("d", robot.targetDist);
            addPacket("Theta Error", robot.drivetrain.getThetaError());
            addPacket("Init Theta", robot.drivetrain.getInitTheta());
            robot.update();

            telemetry.addData("Robot X", robot.x);
            telemetry.addData("Robot Y", robot.y);
            telemetry.addData("Robot Theta", robot.theta);
            telemetry.addData("Turret Theta", robot.turretGlobalTheta);
            telemetry.addData("Shooter Velocity", robot.shooter.getFlywheelVelocity());
            telemetry.update();
        }
    }
}