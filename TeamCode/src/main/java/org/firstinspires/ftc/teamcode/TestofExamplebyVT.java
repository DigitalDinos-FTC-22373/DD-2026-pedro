//This is another example auton based on the Pedro pathing one by Vineeth Talluri

package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Example Auto VT", group = "Examples")
public class TestofExamplebyVT extends LinearOpMode {

    BabyBopBot bbb = new BabyBopBot();

    private Follower follower;
    private final Pose startPose = new Pose(20.261, 120.342, Math.toRadians(142)); // Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = new Pose(58.345, 83.318, Math.toRadians(142)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(15.748, 82.450, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(17.655, 58.677, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(19.732, 34.531, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose(53.600, 111.208); // Final Pose of our robot, off the starting line

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(68.32, 53.38), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierCurve (curved line). */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(68.32, 53.38), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierCurve (curved line). */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(63.09, 31.68), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierCurve (curved line). */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, new Pose(63.09, 31.68), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our leave PathChain. We are using a single path using a BezierLine (straight line).
         * We use Constant Interpolation here instead of Linear*/
        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public Command autoRoutine() {
        return sequential(
                /* Go To Score Command*/
                follow(follower, scorePreload),
                //need to add:put shooter power here
                instant(() -> bbb.setShooterVelocity(1450)),
                waitMs(1500),
                instant(() -> bbb.setShooterVelocity(0)),
                /* Collect 3 Artifacts Command*/
                //need to add:set intake, front and back, and gate ready
                follow(follower, grabPickup1, true),
                instant(() -> {
                    bbb.gateOpen();
                    bbb.setFIntakeInward();
                    bbb.setBIntakeInward();
                }),
                waitMs(1500),
                instant(() ->{
                    bbb.gateClose();
                    bbb.setFIntakeoff();
                    bbb.setBIntakeoff();
                }),




                /* Go Back To Score Command*/
                // need to add:set shooter ready again, also turn front off to save power
                follow(follower, scorePickup1, true),
                waitMs(1500),

                /* Collect 3 Artifacts Command*/
                //need to add:set intake, front and back, and gate ready
                follow(follower, grabPickup2, true),
                waitMs(1500),

                /* Go Back To Score Command*/
                //need to add:set intake, front and back, and gate ready
                follow(follower, scorePickup2, true),
                waitMs(1500),

                /* Collect 3 Artifacts Command*/
                // need to add: gate and intakes ready
                follow(follower, grabPickup3, true),
                waitMs(1500),

                /* Go Back To Score Command*/
                // need to add: shooter commands, as well as intake off, front
                follow(follower, scorePickup3, true),
                waitMs(1500),

                /* Leave Start Line Command*/
                // need to add: make sure everything is reset and ready for teleop
                follow(follower, leave, true),
                waitMs(1500)

        );
    }

    @Override
    public void runOpMode() {
        //These will run when the OpMode is initiated
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        //We schedule all our commands when we start the OpMode
        schedule(autoRoutine());
        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}

