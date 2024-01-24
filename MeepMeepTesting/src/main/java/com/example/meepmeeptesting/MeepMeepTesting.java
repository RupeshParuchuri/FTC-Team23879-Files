package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.robonauts.Constants;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(Constants.R_1_BEGIN_POSE)
                .strafeTo(Constants.R_1_STRAFE_RELEASE_PIXEL_CENTER)
                .splineToLinearHeading(new Pose2d( new Vector2d(-24,-36), 0), Constants.R_1_HEADING_BB)
                //.turn(Math.toRadians(180))
                .strafeTo(new Vector2d(-24, -48))
                //.strafeTo(Constants.R_1_STRAFE_BB)
                //.splineToLinearHeading(Constants.R_1_SPLINE_BB, Constants.R_1_HEADING_BB)
                //.strafeTo(new Vector2d(46, -12))
                //.strafeTo(Constants.R_PARK_BB)
                //.afterDisp(10, pixelDrop.release())
               /* .splineToLinearHeading(new Pose2d( new Vector2d(-36, -12), 0), Constants.R_1_HEADING_BB)
                                .strafeTo(new Vector2d(20, -12))
                //.splineToLinearHeading(Constants.R_1_SPLINE_CENTER, Constants.R_1_HEADING_BB)
                //.strafeTo(Constants.R_1_STRAFE_BB)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.R_1_SPLINE_BB, Constants.R_1_HEADING_BB)
                .strafeTo(new Vector2d(46, -12))

                .strafeTo(Constants.R_PARK_BB)*/

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}