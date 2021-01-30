package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.PI;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
@Disabled
public class Regression extends LinearOpMode {

    public int startX = 90;
    public int startY = 9;
    public double startTheta = PI/2;

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

            if (gamepad1.left_bumper && !flywheelToggle) {
                flywheelToggle = true;
                if (flywheelOn) {
                    robot.shooter.magHome();
                    robot.shooter.flywheelOff();
                } else {
                    robot.shooter.magShoot();
                    robot.shooter.flywheelPS();
                }
                flywheelOn = !flywheelOn;
            } else if (!gamepad1.left_bumper && flywheelToggle) {
                flywheelToggle = false;
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedMid();
            } else {
                robot.shooter.feedHome();
            }

            if (gamepad1.right_trigger > 0) {
                double[] target = robot.shootTargets(Robot.psShoot[0], Robot.psShoot[1], PI / 2, 2);

                if (!(Math.abs(robot.x - target[0]) < xyTol && Math.abs(robot.y - target[1]) < xyTol && Math.abs(robot.theta - target[2]) < thetaTol)) {
                    robot.setTargetPoint(target[0], target[1], target[2]);
                }
            }

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            if (gamepad1.dpad_up) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapPosition() + 0.001);
            }
            if (gamepad1.dpad_down) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapPosition() - 0.001);
            }

            robot.update();

            telemetry.addData("Robot X", robot.x);
            telemetry.addData("Robot Y", robot.y);
            telemetry.addData("Robot Theta", robot.theta);
            telemetry.addData("numRings", robot.numRings);
            telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
            telemetry.addData("Flap Angle", robot.shooter.getFlapPosition());
            telemetry.update();

        }
    }
}