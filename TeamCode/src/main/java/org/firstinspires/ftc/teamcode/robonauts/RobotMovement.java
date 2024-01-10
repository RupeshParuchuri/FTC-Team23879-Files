package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RobotMovement {
    RobotMovement (HardwareMap hardwareMap) {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.actionBuilder(beginPose)
                .lineToX(5)
                .build();
    }
}
