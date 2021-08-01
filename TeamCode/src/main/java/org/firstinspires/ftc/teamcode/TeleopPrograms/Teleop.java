package org.firstinspires.ftc.teamcode.TeleopPrograms;

import static org.firstinspires.ftc.teamcode.RobotClasses.Robot.TurretMode.HIGH_GOAL;
import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Debug.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp(name = "1 Teleop")
@SuppressWarnings("FieldCanBeLocal")
@Config
public class Teleop extends LinearOpMode {

    // Backup Starting Position
    private final int startX = 87;
    private final int startY = 63;
    private final double startTheta = PI/2;

    public static int highGoalClose = 1570;
    public static int highGoalFar = 1640;
    public static int midGoalClose = 1450;
    public static int midGoalFar = 1550;

    private Robot robot;

    public static boolean robotCentric = true;
    public static boolean useAutoPos = true;
    public static boolean isRed = true;

    // Control Gains
    private double xyGain = 1;
    private double wGain = 1;

    // Toggles
    private boolean downToggle = false, armDown = false;
    private boolean clampToggle = false, clamped = true;
    private boolean armIn = true;
    private boolean blockerToggle = false, blockerUp = false;
    private boolean bumperToggle = false;

    /*
    Controller Button Mappings:
    Gamepad 1
    Left Stick/Right Stick - Drivetrain Controls
    B - Bumper Toggle
    X - Reset Odo
    Dpad Up - High Goal Close
    Dpad Down - High Goal Far
    Left Bumper - High Goal Shoot
    Right Bumper - Powershot Shoot
    Left Trigger - Intake Reverse
    Right Trigger - Intake On

    Gamepad 2
    A - Blocker Toggle
    B - Cancel Shoot
    X - Move Turret to Reset
    Y - Pre-Rev Flywheel for High Goal
    Dpad Left - Decrease Theta Offset
    Dpad Right - Increase Theta Offset
    Dpad Up - Increase Velocity Scale
    Dpad Down - Decrease Velocity Scale
    Left Bumper - Wobble Clamp/Unclamp
    Right Bumper - Wobble Up/Down
    Left Trigger - Slow Mode
    Right Trigger - Shoot Override
     */

    @Override
    public void runOpMode() {
        if (useAutoPos) {
            double[] initialData = Logger.readPos();
            telemetry.addData("Starting Position", Arrays.toString(initialData));
            telemetry.update();
            robot = new Robot(this, initialData[1], initialData[2], initialData[3], initialData[4], false, initialData[0] == 1);
            robot.logger.startLogging(false, initialData[0] == 1);
        } else {
            robot = new Robot(this, (isRed ? startX : 144 - startX), startY, startTheta, false, isRed);
            robot.logger.startLogging(false, isRed);
        }

        robot.setLockMode(Robot.TurretMode.HIGH_GOAL);

        waitForStart();

        robot.drivetrain.updateThetaError();

        while (opModeIsActive()) {
            // Intake On / Rev / Off
            if (gamepad1.right_trigger > 0) {
                robot.intake.on();
            } else if (gamepad1.left_trigger > 0) {
                robot.intake.reverse();
            } else {
                robot.intake.off();
            }

            // High Goal / Powershot Shoot
            if (gamepad1.left_bumper) {
                robot.highGoalShoot();
            } else if (gamepad1.right_bumper) {
                robot.powerShotShoot();
            } else if (gamepad1.dpad_up) {
                robot.highGoalShoot(3, robot.turretMode == HIGH_GOAL ? highGoalClose : midGoalClose);
            } else if (gamepad1.dpad_down) {
                robot.highGoalShoot(3, robot.turretMode == HIGH_GOAL ? highGoalFar : midGoalFar);
            }

            // Stop Shoot Sequence
            if (gamepad2.b) {
                robot.cancelShoot();
            }

            // Rev Up Flywheel for High Goal
            if (gamepad2.y) {
                robot.shooter.setFlywheelVelocity(robot.calcHGVelocity());
            }

            // Turret Reset
            if (gamepad2.x) {
                robot.turretReset = true;
            }

            // Wobble Arm In / Up / Down
            if (gamepad2.right_bumper && !downToggle) {
                if (!armIn) {
                    downToggle = true;
                    if (armDown) {
                        robot.wobbleArm.armUp();
                    } else {
                        robot.wobbleArm.armDown();
                    }
                    armDown = !armDown;
                } else {
                    robot.moveWobbleOut = 1;
                    robot.wobbleTime = System.currentTimeMillis();
                    armIn = false;
                    armDown = true;
                }
            } else if (!gamepad2.right_bumper && downToggle) {
                downToggle = false;
            }

            if (gamepad1.b && !bumperToggle) {
                bumperToggle = true;
                robot.intake.bumperReverse = !robot.intake.bumperReverse;
            } else if (!gamepad1.b && bumperToggle) {
                bumperToggle = false;
            }

            if (armIn) {
                robot.intake.autoBumpers(robot.x, robot.y, robot.theta, 12);
            } else if (armDown) {
                robot.intake.bumpersOut();
            } else {
                robot.intake.bumpersHome();
            }

            if (gamepad2.a && !blockerToggle) {
                blockerToggle = true;
                if (blockerUp) {
                    robot.intake.blockerHome();
                } else {
                    robot.intake.blockerVert();
                }
                blockerUp = !blockerUp;
            } else if (!gamepad2.a && blockerToggle) {
                blockerToggle = false;
            }

            // Wobble Clamp / Unclamp
            if (gamepad2.left_bumper && !clampToggle) {
                if (!armIn) {
                    clampToggle = true;
                    if (clamped) {
                        robot.wobbleArm.unClamp();
                    } else {
                        robot.wobbleArm.clamp();
                    }
                    clamped = !clamped;
                }
            } else if (!gamepad2.left_bumper && clampToggle) {
                clampToggle = false;
            }

            // Overrides Shoot in Case it is Taking Too Long
            if (gamepad2.left_trigger > 0) {
                robot.preShootOverride = true;
                robot.shootOverride = true;
            }

            // Slow Mode
            if (gamepad2.right_trigger > 0) {
                xyGain = 0.22;
                wGain = 0.17;
            } else {
                xyGain = 1;
                wGain = 1;
            }

            // Reset Odometry
            if (gamepad1.x) {
                robot.resetOdo(robot.isRed ? 87 : 57, 63, PI/2);
                robot.thetaOffset = 0.03;
                robot.velocityFactor = 0.96;
            }

            // Change Shooting Theta Offset to Compensate for Odometry Drift
            if (gamepad2.dpad_left) {
                robot.thetaOffset -= 0.006;
            } else if (gamepad2.dpad_right) {
                robot.thetaOffset += 0.006;
            }

            // Change Velocity Scaling
            if (gamepad2.dpad_up) {
                robot.velocityFactor += 0.001;
            } else if (gamepad2.dpad_down) {
                robot.velocityFactor -= 0.001;
            }

            // Drivetrain Controls
            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_y * xyGain, -gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
            } else {
                robot.drivetrain.setGlobalControls((isRed ? 1 : -1) * gamepad1.left_stick_y * xyGain, (isRed ? 1 : -1) * gamepad1.left_stick_x * xyGain, -gamepad1.right_stick_x * wGain);
            }

            // Update Robot
            robot.update();

            // Telemetry
            telemetry.addData("X", robot.x);
            telemetry.addData("Y", robot.y);
            telemetry.addData("Theta", robot.theta);
            telemetry.addData("# Rings", robot.numRings);
            telemetry.addData("Theta Offset", robot.thetaOffset);
            telemetry.addData("Shooter Velocity", robot.shooter.getFlywheelVelocity());
            telemetry.addData("# Cycles", robot.cycles);
            telemetry.addData("Average Cycle Time", (robot.cycleTotal / robot.cycles) + "s");
            telemetry.update();
        }

        Log.w("cycle-log", "# Cycles: " + robot.cycles);
        Log.w("cycle-log", "Avg cycle Time: " + (robot.cycleTotal / robot.cycles) + "s");
        if (robot.cycles > 0) {
            Log.w("cycle-log", "Avg dropping longest: " + ((robot.cycleTotal - robot.longestCycle) / (robot.cycles - 1)) + "s");
        }
        robot.stop();
    }
}