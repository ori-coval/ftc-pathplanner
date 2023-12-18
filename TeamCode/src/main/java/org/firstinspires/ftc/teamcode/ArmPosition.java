package org.firstinspires.ftc.teamcode;

public enum ArmPosition {

    SCORE_BOTTOM_CLOSE(0, 0, 0, true),
    SCORE_BOTTOM_MID(0, 0, 0, true),
    SCORE_BOTTOM_FAR(0, 0, 0, true),
    SCORE_MID_CLOSE(0, 0, 0, true),
    SCORE_MID_MID(0, 0, 0, true),
    SCORE_MID_FAR(0, 0, 0, true),
    SCORE_TOP_CLOSE(0, 0, 0, true),
    SCORE_TOP_MID(0, 0, 0, true),
    SCORE_TOP_FAR(0, 0, 0, true),
    SCORE_FRONT_TOP(0,0,0,true),
    SCORE_FRONT_MID(0,0,0,true),
    SCORE_FRONT_BOTTOM(0,0,0,true),
    INTAKE(0, 0, 0, false);

    private final double elevatorHeight;
    private final double extenderLength;
    private final double turretAngle;
    private final boolean elbowIsScoring;

    ArmPosition(double elevatorHeight, double extenderLength, double turretAngle, boolean elbowIsScoring) {
        this.elevatorHeight = elevatorHeight;
        this.elbowIsScoring = elbowIsScoring;
        this.turretAngle = turretAngle;
        this.extenderLength = extenderLength;

    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public double getExtenderLength() {
        return extenderLength;
    }

    public double getTurretAngle(boolean isRightOfBoard) {
        return turretAngle * (isRightOfBoard ? 1 : -1);
    }
}

