package org.firstinspires.ftc.teamcode.Pathing;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

public class Target {
    private double xTarget;
    private double yTarget;
    private double thetaTarget;
    private double vxTarget;
    private double vyTarget;
    private double wTarget;
    private double xKp = MecanumDrivetrain.xKp;
    private double yKp = MecanumDrivetrain.yKp;
    private double thetaKp = MecanumDrivetrain.thetaKp;
    private double xKd = MecanumDrivetrain.xKd;
    private double yKd = MecanumDrivetrain.yKd;
    private double thetaKd = MecanumDrivetrain.thetaKd;

    public Target(Pose pose) {
        xTarget = pose.x;
        yTarget = pose.y;
        thetaTarget = pose.theta;
        vxTarget = pose.vx;
        vyTarget = pose.vy;
        wTarget = pose.w;
    }

    public Target(double xTarget, double yTarget, double thetaTarget) {
        this(new Pose(xTarget, yTarget, thetaTarget, 0, 0, 0));
    }

    public Target theta(double thetaTarget) {
        this.thetaTarget = thetaTarget;
        return this;
    }

    public Target vx(double vxTarget) {
        this.vxTarget = vxTarget;
        return this;
    }

    public Target vy(double vyTarget) {
        this.vyTarget = vyTarget;
        return this;
    }

    public Target w(double wTarget) {
        this.wTarget = wTarget;
        return this;
    }

    public Target xKp(double xKp) {
        this.xKp = xKp;
        return this;
    }

    public Target yKp(double yKp) {
        this.yKp = yKp;
        return this;
    }

    public Target thetaKp(double thetaKp) {
        this.thetaKp = thetaKp;
        return this;
    }

    public Target xKd(double xKd) {
        this.xKd = xKd;
        return this;
    }

    public Target yKd(double yKd) {
        this.yKd = yKd;
        return this;
    }

    public Target thetaKd(double thetaKd) {
        this.thetaKd = thetaKd;
        return this;
    }

    public Pose getPose() {
        return new Pose(xTarget, yTarget, thetaTarget, vxTarget, vyTarget, wTarget);
    }

    public double xKp() {
        return xKp;
    }

    public double yKp() {
        return yKp;
    }

    public double thetaKp() {
        return thetaKp;
    }

    public double xKd() {
        return xKd;
    }

    public double yKd() {
        return yKd;
    }

    public double thetaKd() {
        return thetaKd;
    }
}
