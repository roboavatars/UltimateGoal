package org.firstinspires.ftc.teamcode.Tests.FlickerTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.RobotClasses.Shooter;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.*;

@TeleOp
@Config
//@Disabled
public class DoubleFlickerTest extends LinearOpMode {
    public static double bottomPos = Constants.FEED_MID_POS;
    public static double homePos = Constants.FEED_HOME_POS;
    public static double topPos = Constants.FEED_TOP_POS;
    public static int pos = 1;
    public static boolean magUp = false;
    public static boolean debug = false;
    private double position;

    private boolean magToggle = false;
    private boolean shootToggle = false;
    private long shootTime;
    private int delay = 4;
    public static int period = 250;

    private double x = 87;
    private double y = 63;
    public static double thetaLeft = 1.627;
    public static double thetaMid = 1.571;
    public static double thetaRight = 1.504;

    private Shooter shooter;
    private Intake intake;
    private Robot robot;
    private long meme = 0;
    private boolean iliterallyhavenoideawhatimdoing = true;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "feedServo");
        shooter = new Shooter(this);
        intake = new Intake(this, false);
        robot = new Robot(this, 87, 63, PI/2, false);

        //intake.sticksHalf();
        //intake.sticksUpdate();

        waitForStart();

        while(opModeIsActive()) {
            robot.drivetrain.setControls(-gamepad1.left_stick_y , -gamepad1.left_stick_x , -w.right_stick_x );
            if (debug) {
                if (pos == 0) {
                    position = bottomPos;
                } else if (pos == 1) {
                    position = homePos;
                } else if (pos == 2) {
                    position = topPos;
                }
                servo.setPosition(position);

                if (magUp) {
                    shooter.magShoot();
                } else {
                    shooter.magHome();
                }
            } else {
                // Intake on/off/rev
                if(gamepad2.a){
                    intake.blockerDown();
                }
                else{
                    intake.blockerUp();
                }
                if (gamepad2.right_trigger > 0) {
                    intake.on();
                    robot.wobbleArm.setPower(-gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0) {
                    intake.reverse();
                    if (gamepad2.a){
                        robot.wobbleArm.setPower(1);
                    }
                    else{robot.wobbleArm.setPower(0.7);}
                } else {
                    intake.off();
                    robot.wobbleArm.setPower(0);
                }
                telemetry.clearAll();
                telemetry.addData("delaytime", meme);
                telemetry.addData("elapsedtime", System.currentTimeMillis());
                /*if ((System.currentTimeMillis()-meme) >= 500){
                    meme = System.currentTimeMillis();

                    if (iliterallyhavenoideawhatimdoing){
                        intake.blockerDown();

                        telemetry.addData("front", 0);
                        iliterallyhavenoideawhatimdoing = false;

                    }
                    else{
                        intake.blockerUp();
                        iliterallyhavenoideawhatimdoing = true;
                        telemetry.clearAll();
                        telemetry.addData("front", 0);
                    }
                    telemetry.update();
                }*/


            }

            addPacket("delay", delay);
            robot.update();
        }
    }
}