package org.firstinspires.ftc.teamcode;

public enum ArmPosition {

    SCORE_BOTTOM_CLOSE(0, 0, 0, 0),
    SCORE_BOTTOM_FAR(0, 0, 0, 0),
    SCORE_MID_CLOSE(0, 0, 0, 0),
    SCORE_MID_FAR(0, 0, 0, 0),
    SCORE_TOP_CLOSE(0, 0, 0, 0),
    SCORE_TOP_FAR(0, 0, 0, 0),
    SCORE_FRONT_TOP(0,0,0,0),
    SCORE_FRONT_MID(0,0,0,0),
    SCORE_FRONT_BOTTOM(0,0,0,0),
    INTAKE(0, 0, 0,0);

    private final double elevatorHeight;
    private final double extenderLength;
    private final double turretAngle;
    private final double elbowAngle;

    ArmPosition(double elevatorHeight, double extenderLength, double turretAngle, double elbowAngle) {
        this.elevatorHeight = elevatorHeight;
        this.elbowAngle = elbowAngle;
        this.turretAngle = turretAngle;
        this.extenderLength = extenderLength;

    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public double getExtenderLength() {
        return extenderLength;
    }

    public double getTurretAngle(boolean isLeftOfBoard) {return turretAngle * (isLeftOfBoard ? -1 : 1);
    }

    public double getElbowAngle() {return elbowAngle;}
}

