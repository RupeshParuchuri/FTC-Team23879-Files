package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RobotMovement {
    Telemetry telemetry = null;
    Pose2d beginPose = null;
    MecanumDrive drive = null;

    public RobotMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.beginPose = new Pose2d(0, 0, 0);
        this.drive = new MecanumDrive(hardwareMap, beginPose);

    }

    public RobotMovement(HardwareMap hardwareMap, Telemetry telemetry, Pose2d beginPose) {
        this.telemetry = telemetry;
        this.beginPose = beginPose;
    }

    public MecanumDrive moveToSpike(int x, int y) {
        Pose2d pose = new Pose2d(0, 0, 0);

        Actions.runBlocking(drive.
                actionBuilder(pose).
                lineToXConstantHeading(x)
                .build());
        return drive;
    }

//    public void trajectorySample() {
//        Pose2d pose = new Pose2d(0, 0, 0);
//
//        Actions.runBlocking(drive.
//                actionBuilder(pose).lineToX(-12)
//
//    }

    public MecanumDrive rotate(int i) {
        return null;
    }
}
