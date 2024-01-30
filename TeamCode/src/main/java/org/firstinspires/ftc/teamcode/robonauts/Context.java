package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Context {
    Pose2d currentPose = new Pose2d(new Vector2d(0, 0), 0);
    String robotState="";
    double linearSlidePosition;
    double armPosition;
    double mainclawposition;

    String clawMainSource;

    public String getClawMainSource() {
        return clawMainSource;
    }

    public void setClawMainSource(String clawMainSource) {
        this.clawMainSource = clawMainSource;
    }

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

    public double getArmPosition() {
        return armPosition;
    }

    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
    }

    public double getMainclawposition() {
        return mainclawposition;
    }

    public void setMainclawposition(double mainclawposition) {
        this.mainclawposition = mainclawposition;
    }

    public double getLinearSlidePosition() {
        return linearSlidePosition;
    }

    public void setLinearSlidePosition(double linearSlidePosition) {
        this.linearSlidePosition = linearSlidePosition;
    }
}
