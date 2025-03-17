package frc.robot;

public class Pose {
    public double x, y, yaw;

    public Pose(double x, double y, double yaw) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
    }

    public Pose() {
        this.x = 0;
        this.y = 0;
        this.yaw = 0;
    }

}
