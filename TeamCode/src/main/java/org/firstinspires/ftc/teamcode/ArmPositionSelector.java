package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmPositionSelector {
    public static ArmPosition[][] sideScorePositions = {
            {ArmPosition.SCORE_TOP_CLOSE, ArmPosition.SCORE_TOP_FAR},
            {ArmPosition.SCORE_MID_CLOSE, ArmPosition.SCORE_MID_FAR},
            {ArmPosition.SCORE_BOTTOM_CLOSE, ArmPosition.SCORE_BOTTOM_FAR}
    };
    public static ArmPosition[] frontScorePositions = {ArmPosition.SCORE_TOP_FRONT, ArmPosition.SCORE_MID_FRONT, ArmPosition.SCORE_BOTTOM_FRONT};
    private static int selectedScoreHeight = 1;
    private static boolean isSelectedScoreSideLeft = false;
    private static Side selectedRobotSide = Side.CENTER;

    public static void setRobotSide(Side side) {
        selectedRobotSide = side;
    }

    public static Side getSelectedRobotSide() {
        return selectedRobotSide;
    }

    public static void moveRight() {
        isSelectedScoreSideLeft = false;}

    public static void moveLeft() {
        isSelectedScoreSideLeft = true;}

    public static void moveUp() {
        if (selectedScoreHeight != 0) {
            selectedScoreHeight--;
        }
    }

    public static void moveDown() {
        if (selectedScoreHeight != 2) {
            selectedScoreHeight++;
        }
    }

    public static ArmPosition getPosition() {
        if (selectedRobotSide == Side.CENTER){
            return frontScorePositions[selectedScoreHeight];
        }
        boolean isClose = ((selectedRobotSide == Side.LEFT) == isSelectedScoreSideLeft);
        return sideScorePositions[selectedScoreHeight][isClose?0:1];
    }

    public static void telemetry(Telemetry telemetry) {
        for (int yCounter = 0; yCounter < 3; yCounter++ ) {
            String tempStr = "";
            for (int xCounter = 0; xCounter < 2; xCounter++) {
                if (xCounter == (isSelectedScoreSideLeft?0:1) && yCounter == selectedScoreHeight) {
                    tempStr += "X";
                } else {
                    tempStr += "O";
                }
            }
            telemetry.addLine(tempStr);
        }
        if (selectedRobotSide.equals(Side.RIGHT)) {
            telemetry.addLine("__^");
        } else if (selectedRobotSide.equals(Side.CENTER)) {
            telemetry.addLine("_^_");
        } else {
            telemetry.addLine("^__");
        }
    }
}

