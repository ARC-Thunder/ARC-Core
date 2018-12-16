package org.firstinspires.ftc.teamcode.detectgold;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Accelerometer;

public class GoldDetection {

    private final double CAM_FOCAL_LENGTH, GOLD_WIDTH_IN, MAX_TRAVEL, CAMERA_HEIGHT, CAMERA_DISTANCE_FROM_FRONT;

    private ThunderGoldAlignDetector detector;

    private Dogeforia vuforia;

    private Accelerometer accelerometer;

    public GoldDetection(double camFocalLength, double goldWidthIn, double maxTravelIn, double cameraHeight, double camDistanceFromFront, HardwareMap hardwareMap, Dogeforia vuforia) {
        CAM_FOCAL_LENGTH = camFocalLength;
        GOLD_WIDTH_IN = goldWidthIn;
        MAX_TRAVEL = maxTravelIn;
        CAMERA_HEIGHT = cameraHeight;
        CAMERA_DISTANCE_FROM_FRONT = camDistanceFromFront;

        this.vuforia = vuforia;

        detector = new ThunderGoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        //CameraDevice.getInstance().init(CameraDevice.CAMERA_DIRECTION.CAMERA_DIRECTION_FRONT);
        CameraDevice.getInstance().setFlashTorchMode(true);

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        //vuforia.showDebug(); //Don't enable this since it causes a crash
        vuforia.start();

        accelerometer = new Accelerometer(hardwareMap);
    }

    public double[] getGoldOffset() {

        double XZ_Hypotenuse = distanceFromGold(detector.getBestRectWidth()); // The hypotenuse of the triangle (located in the XZ plane)

        while (!detector.isFound() || Double.isInfinite(XZ_Hypotenuse) || detector.bestRectIsNull()) {
            if (Thread.interrupted()) {
                detector.disable();
                double[] retData = {Double.MIN_VALUE, Double.MIN_VALUE};
                return retData;
            }

            XZ_Hypotenuse = distanceFromGold(detector.getBestRectWidth()); // The hypotenuse of the triangle (located in the XZ plane)
        }
            /*
         TOP VIEW (FROM BEHIND ROBOT)

                  X AXIS
                    /\
                    ||
                    ||
            <================> Y AXIS
                    ||
                    ||
                    \/

             HEIGHT OR 3RD DIMENSION: Z AXIS

             */

        double cubeDistance = cubeDistanceFromCenter(detector.getBestRectWidth()); // The distance (along the Y axis) to the X axis

        double distanceAlongXAxis = Math.sqrt(Math.abs(Math.pow(XZ_Hypotenuse, 2) - Math.pow(CAMERA_HEIGHT, 2))); // The distance (along the X axis) to the point at which the XZ hypotenuse intersects the XY plane

        double angle = Math.toDegrees(Math.atan2(cubeDistance, distanceAlongXAxis)); // The angle to turn, in degrees. Negative = clockwise, positive = counterclockwise

            /*
       TRIANGLE ON THE XY PLANE:

           cubeDistance
             _________ [GOLD]
           d |       /
           i |      / t
           s |     / r
           t |    / a
           a |   / v
           n |  / e
           c | / l
           e |/
           A ⚪ -> Horizontal center of the front of the robot
           l
           o
           n
           g
           X
           A
           x
           i
           s

             */

        double distanceToTravel = Math.min((int) (Math.sqrt(Math.pow(cubeDistance, 2) + Math.pow(distanceAlongXAxis, 2)) - CAMERA_DISTANCE_FROM_FRONT / 2), MAX_TRAVEL); //Use the pythagorean theorem to calculate the length of the hypotenuse. Always rounds up to an integer to ensure that the robot will reach the gold every time
        //In case the phone reads a huge distance, it will reduce it to sqrt(24^2 + 24^2)

        Accelerometer.PhoneRotation rotation = accelerometer.getPhoneRotation();

        int roundedAngle = (angle >= 0) ? (int) (angle + 0.5) : (int) (angle - 0.5); //Round to the nearest integer

        roundedAngle *= (rotation == Accelerometer.PhoneRotation.UP) ? -1 : 1;

        double[] returnData = {distanceToTravel, roundedAngle};

        detector.disable();
        accelerometer.stop();

        return returnData;
    }

    private double calculateFocalLength(int goldWidthPX, int distanceFromObjIn) {
        return goldWidthPX * distanceFromObjIn / GOLD_WIDTH_IN;
    }

    private double distanceFromGold(int goldWidthPX) {
        return GOLD_WIDTH_IN * CAM_FOCAL_LENGTH / goldWidthPX;
    }

    private double cubeDistanceFromCenter(double goldWidthPX) {
        return GOLD_WIDTH_IN * detector.distanceToVerticalCenter() / goldWidthPX; //Solve the ratio
    }

}
