package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Context {
    Pose2d currentPose = new Pose2d(new Vector2d(0, 0), 0);
    String robotState="";

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public String getRobotState() {
        return robotState;
    }

    public void setRobotState(String robotState) {
        this.robotState = robotState;
    }
}
