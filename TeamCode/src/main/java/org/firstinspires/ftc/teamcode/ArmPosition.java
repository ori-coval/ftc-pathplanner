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

    SCORE_TOP_FAR(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.intake),
    SCORE_TOP_FRONT(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.intake),
    SCORE_TOP_CLOSE(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.intake),
    SCORE_MID_FAR(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.intake),
    SCORE_MID_FRONT(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.intake),
    SCORE_MID_CLOSE(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.intake),
    SCORE_BOTTOM_FAR(0,Extender.Position.CLOSED,0,0, 0, Cluster.intake),
    SCORE_BOTTOM_FRONT(0,Extender.Position.CLOSED,0,0, 0, Cluster.intake),
    SCORE_BOTTOM_CLOSE(0,Extender.Position.CLOSED,0,0, 0, Cluster.intake),
    INTAKE(0, Extender.Position.CLOSED, 0,0.06, 0, Cluster.intake),
    SAFE_PLACE(10, Extender.Position.CLOSED,0,0.4, 0, Cluster.intake),
    TEST_POSITION(10, Extender.Position.CLOSED, -30, 0.4, 0, Cluster.scoring),
    SECOND_TEST_POSITION(20, Extender.Position.OPEN, 30, 0.6, 0.4, Cluster.scoring);

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

    public double getTurretAngle(boolean isLeftOfBoard) {return turretAngle * (isLeftOfBoard ? -1 : 1);
    }

    public double getElbowPosition() {return elbowPosition;}
    public double getAntiTurretPosition() {
        return antiTurretPosition;
    }

    public Cluster getCluster(){
        return cluster;
    }
}

