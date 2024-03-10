package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public enum ArmPosition {

    /*

    NOT UPDATED: ('cause no one got the time to do it (and also I'm alone here so there's no one to explain to))

    clusters- each cluster is a group containing multiple positions that


    Assuming the robot is in the right selectedRobotSide of the board:

          far  close
           |      |
           v      v
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- top
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- mid
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- bottom

     Robot's on the left selectedRobotSide:

   close  far
     |     |
     v     v
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- top
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- mid
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- bottom

    Robot's on the front of the board:

         front
           |
           v
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- top
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- mid
     isSelectedScoreSideLeft     isSelectedScoreSideLeft      isSelectedScoreSideLeft  <- bottom

     */

    SCORE_TOP_FAR(34.1, Extender.Position.OPEN, 42, 0.42, 0.85, Cluster.scoring), //Can't physically reach this
    SCORE_TOP_FRONT(29.361, Extender.Position.OPEN, 0, 0.47, 0.7, Cluster.scoring),
    SCORE_TOP_CLOSE(34.1, Extender.Position.MID_WAY, 44, 0.42, 0.9, Cluster.scoring),
    SCORE_MID_FAR_2(27.346 + 10, Extender.Position.MID_WAY, 55, 0.336, 0.88, Cluster.scoring),
    SCORE_MID_FAR(27.346, Extender.Position.MID_WAY, 55, 0.336, 0.88, Cluster.scoring),
    SCORE_MID_FRONT(17.464, Extender.Position.MID_FRONT, 0, 0.45, 0.7, Cluster.scoring),
    SCORE_MID_CLOSE(22.35, Extender.Position.CLOSED, 42, 0.336, 0.8, Cluster.scoring),
    SCORE_BOTTOM_FAR(12.59, Extender.Position.MID_WAY, 57, 0.336, 0.9, Cluster.scoring),
    SCORE_BOTTOM_FRONT(15.48, Extender.Position.BOTTOM_FRONT, 0, 0.34, 0.7, Cluster.scoring),
    SCORE_AUTO_BOTTOM_FAR_2_RED(22.59 + 10, Extender.Position.OPEN, 70, 0.336, 1, Cluster.scoring),
    SCORE_AUTO_BOTTOM_FAR_2_BLUE(20.714 + 10, Extender.Position.OPEN, 59, 0.336, 0.92, Cluster.scoring),
    SCORE_AUTO_BOTTOM_FAR_RED(22.59, Extender.Position.OPEN, 70, 0.336, 1, Cluster.scoring),
    SCORE_AUTO_BOTTOM_FAR_BLUE(20.714, Extender.Position.OPEN, 62, 0.336, 0.92, Cluster.scoring),
    SCORE_AUTO_BOTTOM_MID_2_RED(12.59 + 10, Extender.Position.MID_WAY, 54, 0.336, 1, Cluster.scoring),
    SCORE_AUTO_BOTTOM_MID_2_BLUE(24.59 + 10, Extender.Position.MID_WAY, 72, 0.336, 1, Cluster.scoring),
    SCORE_AUTO_BOTTOM_MID_RED(12.59, Extender.Position.MID_WAY, 54, 0.336, 1, Cluster.scoring),
    SCORE_AUTO_BOTTOM_MID_BLUE(24.59, Extender.Position.MID_WAY, 72, 0.336, 1, Cluster.scoring),
    SCORE_BOTTOM_CLOSE_2_RED(12.34 + 10, Extender.Position.CLOSED, 50, 0.34, 0.9, Cluster.scoring),
    SCORE_BOTTOM_CLOSE_2_BLUE(12.34 + 10, Extender.Position.CLOSED, 72, 0.34, 0.9, Cluster.scoring),
    SCORE_BOTTOM_CLOSE_RED(12.34, Extender.Position.CLOSED, 50, 0.34, 0.9, Cluster.scoring),
    SCORE_BOTTOM_CLOSE_BLUE(12.34, Extender.Position.CLOSED, 72, 0.34, 0.9, Cluster.scoring),
    INTAKE(0, Extender.Position.CLOSED, 0,0.001, 0.03, Cluster.intake),
    AUTO_INTAKE(0, Extender.Position.CLOSED, 0,0.02, 0.03, Cluster.intake),
    AUTONOMOUS_PURPLE_PIXEL(0, Extender.Position.AUTONOMOUS_PURPLE, 0, 0.25, 0, Cluster.intake),
    SAFE_PLACE(12.32, Extender.Position.CLOSED,0,0.45, 0.7, Cluster.intake),
    SCORING(12.32, Extender.Position.CLOSED, 90, 0.45, 0.9, Cluster.scoring),
    SCORING_AUTO(12.32, Extender.Position.CLOSED, 110, 0.45, 0.9, Cluster.scoring);/*,
    //Autonomous
    SCORE_AUTONOMOUS_CLOSE(),
    SCORE_AUTONOMOUS_FAR(),
    SCORE_AUTONOMOUS_FRONT()*/

    private final double elevatorHeight;
    private final Extender.Position extenderPosition;
    private final double turretAngle;
    private final double elbowPosition;
    private final double antiTurretPosition;
    private final Cluster cluster;

    private enum Cluster{
        intake,
        scoring
    }
    ArmPosition(double elevatorHeight, Extender.Position extenderPosition, double turretAngle, double elbowPosition, double antiTurretPosition, Cluster cluster) {
        this.elevatorHeight = elevatorHeight;
        this.elbowPosition = elbowPosition;
        this.turretAngle = turretAngle;
        this.extenderPosition = extenderPosition;
        this.antiTurretPosition = antiTurretPosition;
        this.cluster = cluster;
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public Extender.Position getExtenderPosition() {
        return extenderPosition;
    }

    public double getTurretAngle(boolean isLeftOfBoard) {
        return turretAngle * (isLeftOfBoard ? -1 : 1);
    }

    public double getElbowPosition() {
        return elbowPosition;
    }
    public double getAntiTurretPosition(boolean isLeftOfBoard) {
        if(!isLeftOfBoard & (antiTurretPosition > 1./3)) {
            return (antiTurretPosition + ((antiTurretPosition > 2./3) ? (-2) : 2) * Math.abs(2./3 - antiTurretPosition)) - 0.02;
        } else return antiTurretPosition + 0.02;
    }

    public Cluster getCluster(){
        return cluster;
    }
}

