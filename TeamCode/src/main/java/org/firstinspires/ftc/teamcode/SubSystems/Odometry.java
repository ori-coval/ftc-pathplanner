package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry extends SubsystemBase {
    private DcMotor odometryX;
    private DcMotor odometryY;
    private Translation2d location;
    private final double CIRCUMFERENCE = 0.048 * Math.PI;
    private final double TICKS_PER_REV = 2000;
    private final double TICKS_PER_METER = TICKS_PER_REV / CIRCUMFERENCE;
    private double offsetX = 0;
    private double offsetY = 0;
    public Odometry(DcMotor odometryX, DcMotor odometryY) {
        this.odometryX = odometryX;
        this.odometryY = odometryY;
        location = new Translation2d();
    }

    public Translation2d getLocation() {
        return location;
    }
    private void update() {
        location = new Translation2d(
                (odometryX.getCurrentPosition() / TICKS_PER_METER) - offsetX,
                (odometryY.getCurrentPosition() / TICKS_PER_METER) - offsetY
        ) {
            @Override
            public String toString() {
                return getX() + " " + getY();
            }
        };
    }
    public void resetLocation(Translation2d newLocation){
        offsetX += location.getX() - newLocation.getX();
        offsetY += location.getY() - newLocation.getY();
    }

    public void resetLocation(){
        resetLocation(new Translation2d(0,0));
    }
    @Override
    public void periodic() {
        this.update();
    }
}
