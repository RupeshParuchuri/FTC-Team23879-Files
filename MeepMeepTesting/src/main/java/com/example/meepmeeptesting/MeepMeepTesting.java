package com.example.meepmeeptesting;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(Constants.R_1_BEGIN_POSE)
                .strafeTo(Constants.R_1_STRAFE_RELEASE_PIXEL_LEFT)
                        .strafeTo(Constants.R_1_STRAFE_BACK)
                        .strafeTo(Constants.R_1_STRAFE_BB)
                        .splineToLinearHeading(Constants.R_1_SPLINE_BB, Constants.R_1_HEADING_BB)
                        .strafeTo(Constants.R_PARK_BB)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}