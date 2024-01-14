package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmPositionSelector {
    public static ArmPosition[][] sideScorePositions = {
            {ArmPosition.SCORE_TOP_CLOSE, ArmPosition.SCORE_TOP_FAR},
            {ArmPosition.SCORE_MID_CLOSE, ArmPosition.SCORE_MID_FAR},
            {ArmPosition.SCORE_BOTTOM_CLOSE, ArmPosition.SCORE_BOTTOM_FAR}
    };
    public static ArmPosition[] frontScorePositions = {ArmPosition.SCORE_BOTTOM_FRONT, ArmPosition.SCORE_MID_FRONT, ArmPosition.SCORE_TOP_FRONT};
    /*
        TODO: Left  and Right - isSelectedScoreSideLeft can be 0 or 2 (close or far)
              Front - isSelectedScoreSideLeft can be only 1 (Front). which means only selectedScoreHeight can change
              These limitations needs to happen in the move methods
              for example if the selectedRobotSide is Left then jump 2 each press in the code.
              do these limitations in the code only,
              the telemetry view has to stay simple, and constant. In order to not confuse the 2nd driver.
              In telemetry add another line that indicates which column the user can use to score
              for example use ^ under the XXX
              use buttons to choose which selectedRobotSide the robot is in: Left, Front or Right.
              When a button is pressed, reset isSelectedScoreSideLeft and selectedScoreHeight.
    */

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
        if (selectedScoreHeight != 2) {
            selectedScoreHeight++;
        }
    }

    public static void moveDown() {
        if (selectedScoreHeight != 0) {
            selectedScoreHeight--;
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
        for (int yCounter = 2; yCounter >= 0; yCounter-- ) {
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
        telemetry.update();
    }
}

