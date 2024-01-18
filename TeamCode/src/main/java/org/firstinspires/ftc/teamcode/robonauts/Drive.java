package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Drive {
    private MecanumDrive mecanumDrive;
    Pose2d pose2d;



    public Drive(MecanumDrive mecanumDrive, HardwareMap hardwareMap, Pose2d beginPose) {
        this.mecanumDrive = mecanumDrive;
        this.pose2d=beginPose;
    }

    /*public Action turn(double angle) {
        return new TodoAction();
    }*/

    public Action strafeTo(double x, double y, Pose2d pose2d) {
        if (pose2d != null) {
            this.pose2d=pose2d;
        }
        return mecanumDrive.
                actionBuilder(pose2d).strafeTo(new Vector2d(x, y))
                .build();
    }
    public Action turn(double angle, Pose2d pose2d) {
        return mecanumDrive.
                actionBuilder(pose2d).turn(angle)
                .build();
    }

    public Action toX(double x, Pose2d pose2d1) {
        return mecanumDrive.
                actionBuilder(pose2d1).lineToX(x)
                .build();
    }
    public Action toY(double y, Pose2d pose2d1) {
        return mecanumDrive.
                actionBuilder(pose2d1).lineToX(y)
                .build();
    }
    public Pose2d getCurrentPose() {
        return pose2d;
    }

    public void setCurrentPose(Pose2d pose) {
        pose2d = pose;
    }
}

