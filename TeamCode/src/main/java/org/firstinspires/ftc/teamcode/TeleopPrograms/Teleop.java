package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Config
@TeleOp
public class Teleop extends LinearOpMode {

    public static int startX = 57;
    public static int startY = 135;
    public static double startTheta = Math.PI/2;

    private Robot robot;
    private final boolean robotCentric = true;

    double targetShooterVelocity = 0;

    public boolean a = false;
    public boolean b = false;
    public boolean y = false;
    public boolean x = false;

    @Override
    public void runOpMode() {
        //hi

        /*double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition));
        telemetry.update();*/

        robot = new Robot(this, startX, startY, startTheta); // Robot(this, initialPosition[0], initialPosition[1], initialPosition[2])
        // robot.logger.startLogging();
//        robot.intake.intakeOn();
        robot.t265.startCam();

        waitForStart();

        while(opModeIsActive()) {

            if (a && gamepad1.a) {
                robot.intake.intakeOff();
                a = false;
            } else if (!a && gamepad1.a) {
                robot.intake.intakeOn();
                a  = true;
            }

            if (gamepad1.left_bumper) {
                robot.shooter.feedHome();
                robot.shooter.magShoot();
                robot.shooter.setShooterVelocity(-2000);
            } else {
                robot.shooter.feedHome();
                robot.shooter.magHome();
                robot.shooter.flywheelOff();
            }

            if (gamepad1.right_bumper) {
                robot.shooter.feedShoot();
            } else {
                robot.shooter.feedHome();
            }

            if (gamepad1.x) {
                targetShooterVelocity += 1;
                robot.shooter.setShooterVelocity(targetShooterVelocity);
            } else if (gamepad1.b) {
                targetShooterVelocity -= 1;
                robot.shooter.setShooterVelocity(targetShooterVelocity);
            }

            if (gamepad1.dpad_up && robot.shooter.getFlapAngle() < 0.22) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() + 0.001);
            }
            if (gamepad1.dpad_down && robot.shooter.getFlapAngle() > 0.03) {
                robot.shooter.flapServo.setPosition(robot.shooter.getFlapAngle() - 0.001);
            }

            if (gamepad1.right_trigger > 0) {
                double[] target = robot.shoot(3);
                robot.drivetrain.setTargetPoint(target[0], target[1], target[2]);
                robot.shooter.setFlapAngle(target[3]);
            } else if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            robot.update();

            telemetry.addData("Shooter Velocity", robot.shooter.getShooterVelocity());
            telemetry.addData("Target Shooter Velocity", targetShooterVelocity);
            telemetry.addData("Flap Angle", robot.shooter.getFlapAngle());
            telemetry.addData("Robot X", robot.drivetrain.x);
            telemetry.addData("Robot Y", robot.drivetrain.y);
            telemetry.addData("Robot Theta", robot.drivetrain.theta);
            telemetry.update();

        }
        // robot.logger.stopLogging();
        robot.t265.stopCam();
    }
}