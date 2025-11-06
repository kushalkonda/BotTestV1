package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class ShooterController extends LinearOpMode {
    private DcMotorEx shooter;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 2.831, 9, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -65, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double myX = 0;
            double myY = 0;
            double myZ = 0;

            double myPitch = 0;
            double myRoll = 0;
            double myYaw = 0;



            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                     myX = detection.robotPose.getPosition().x;
                     myY = detection.robotPose.getPosition().y;
                     myZ = detection.robotPose.getPosition().z;

                     myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                     myRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                     myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop












            double ballX = myX + .63;
            double ballY = myY;
            double ballZ = myZ+12.8;



            double launchAngle = 65;

            double goalDistance = d2d(-60,-63,myX,myY);
            double goalHeight = 38.74;

            double launchVelocity = Math.sqrt(
                    (-386.1*Math.pow(goalDistance,2))/
                            (2*Math.cos(Math.toRadians(65)) *
                                    (goalDistance*Math.tan(Math.toRadians(65)) - (goalHeight-ballZ))
                            )
            );

            double motorRPM = (60*launchVelocity)/(2*Math.PI*.9*3.78);

            shooter.setVelocity(motorRPM,AngleUnit.RADIANS);



            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();
            sleep(20);

        }

        visionPortal.close();

    }
    private double d2d(double goalX,double goalY,double robotX,double robotY){
        return Math.sqrt(
                Math.pow(goalX-robotX,2) + Math.pow(goalY-robotY,2)
        );
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()


}