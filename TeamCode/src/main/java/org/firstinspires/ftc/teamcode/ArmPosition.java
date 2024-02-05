package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.SubSystems.Extender;

public enum ArmPosition {

    /*
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

    SCORE_TOP_FAR(10, Extender.Position.CLOSED, -30, 0.4, 0, Cluster.scoring), //Can't physically reach this
    SCORE_TOP_FRONT(29.361, Extender.Position.OPEN, 0, 0.47, 0.7, Cluster.scoring),
    SCORE_TOP_CLOSE(29.1, Extender.Position.OPEN, 35, 0.524, 0.8, Cluster.scoring),
    SCORE_MID_FAR(31.26, Extender.Position.OPEN, 55.245, 0.3633, 0.8616, Cluster.scoring),
    SCORE_MID_FRONT(27.464, Extender.Position.OPEN, 0, 0.405, 0.7, Cluster.scoring),
    SCORE_MID_CLOSE(31, Extender.Position.MID_WAY, 39.19, 0.395, 0.8616, Cluster.scoring),
    SCORE_BOTTOM_FAR(18.276, Extender.Position.OPEN, -62.52, 0.34, 0.9, Cluster.scoring),
    SCORE_BOTTOM_FRONT(15.48, Extender.Position.MID_WAY, 0, 0.34, 0.7, Cluster.scoring),
    SCORE_BOTTOM_CLOSE(18.33, Extender.Position.CLOSED, -50.61, 0.34, 0.9, Cluster.scoring),
    INTAKE(0, Extender.Position.CLOSED, 0,0.06, 0, Cluster.intake),
    SAFE_PLACE(10, Extender.Position.CLOSED,0,0.4, 0, Cluster.intake),
    SCORING(10, Extender.Position.CLOSED, 90, 0.4, 0, Cluster.scoring),
    TEST_POSITION(10, Extender.Position.CLOSED, -30, 0.4, 0, Cluster.scoring),
    SECOND_TEST_POSITION(30, Extender.Position.OPEN, 90, 0.6, 0.4, Cluster.scoring),
    THIRD_TEST_POSITION(30, Extender.Position.OPEN, 0, 0.7, 0.4, Cluster.scoring);
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
    public double getAntiTurretPosition() {
        return antiTurretPosition;
    }

    public Cluster getCluster(){
        return cluster;
    }
}

