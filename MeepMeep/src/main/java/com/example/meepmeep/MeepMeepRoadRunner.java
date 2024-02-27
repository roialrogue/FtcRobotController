package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepRoadRunner {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(227.80474018552286), Math.toRadians(222.7444266666667), 13.11)
                .followTrajectorySequence(drive -> //38
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -61, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-32, -33, Math.toRadians(0)))

                    .lineToLinearHeading(new Pose2d(-36, -58, Math.toRadians(0)))

                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))

                    .splineToConstantHeading(new Vector2d(46, -35), Math.toRadians(0))
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}