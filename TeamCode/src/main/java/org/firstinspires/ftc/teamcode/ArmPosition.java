package org.firstinspires.ftc.teamcode;

public enum ArmPosition {

    /*
    clusters- each cluster is a group containing multiple positions that


    Assuming the robot is in the right side of the board:

          far  close
           |      |
           v      v
     X     X      X  <- top
     X     X      X  <- mid
     X     X      X  <- bottom

     Robot's on the left side:

   close  far
     |     |
     v     v
     X     X      X  <- top
     X     X      X  <- mid
     X     X      X  <- bottom

    Robot's on the front of the board:

         front
           |
           v
     X     X      X  <- top
     X     X      X  <- mid
     X     X      X  <- bottom

     */

    SCORE_TOP_FAR(0, 0, 0, 0, Cluster.cluster0),
    SCORE_TOP_FRONT(0, 0, 0, 0, Cluster.cluster0),
    SCORE_TOP_CLOSE(0, 0, 0, 0, Cluster.cluster0),
    SCORE_MID_FAR(0, 0, 0, 0, Cluster.cluster0),
    SCORE_MID_FRONT(0, 0, 0, 0, Cluster.cluster0),
    SCORE_MID_CLOSE(0, 0, 0, 0, Cluster.cluster0),
    SCORE_BOTTOM_FAR(0,0,0,0, Cluster.cluster0),
    SCORE_BOTTOM_FRONT(0,0,0,0, Cluster.cluster0),
    SCORE_BOTTOM_CLOSE(0,0,0,0, Cluster.cluster0),
    INTAKE(0, 0, 0,0, Cluster.cluster0),
    SAFE_PLACE(0,0,0,0,Cluster.cluster0);

    private final double elevatorHeight;
    private final double extenderLength;
    private final double turretAngle;
    private final double elbowAngle;
    private final Cluster cluster;


    private enum Cluster{
        cluster0,
        cluster1
    }
    ArmPosition(double elevatorHeight, double extenderLength, double turretAngle, double elbowAngle, Cluster cluster) {
        this.elevatorHeight = elevatorHeight;
        this.elbowAngle = elbowAngle;
        this.turretAngle = turretAngle;
        this.extenderLength = extenderLength;
        this.cluster = cluster;
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
    public Cluster getCluster(){
        return cluster;
    }
}

