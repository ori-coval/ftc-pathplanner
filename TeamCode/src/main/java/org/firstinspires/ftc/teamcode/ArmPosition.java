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

    SCORE_TOP_FAR(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.cluster0),
    SCORE_TOP_FRONT(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.cluster0),
    SCORE_TOP_CLOSE(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.cluster0),
    SCORE_MID_FAR(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.cluster0),
    SCORE_MID_FRONT(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.cluster0),
    SCORE_MID_CLOSE(0, Extender.Position.CLOSED, 0, 0, 0, Cluster.cluster0),
    SCORE_BOTTOM_FAR(0,Extender.Position.CLOSED,0,0, 0, Cluster.cluster0),
    SCORE_BOTTOM_FRONT(0,Extender.Position.CLOSED,0,0, 0, Cluster.cluster0),
    SCORE_BOTTOM_CLOSE(0,Extender.Position.CLOSED,0,0, 0, Cluster.cluster0),
    INTAKE(0, Extender.Position.CLOSED, 0,0.06, 0, Cluster.cluster0),
    SAFE_PLACE(10, Extender.Position.CLOSED,0,0.4, 0, Cluster.cluster0),
    TEST_POSITION(10, Extender.Position.CLOSED, 90, 0.4, 0, Cluster.cluster0);

    private final double elevatorHeight;
    private final Extender.Position extenderPosition;
    private final double turretAngle;
    private final double elbowPosition;
    private final double antiTurretPosition;
    private final Cluster cluster;


    private enum Cluster{
        cluster0,
        cluster1
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

