package org.firstinspires.ftc.teamcode.detectgold;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "Focal Length Calculator", group = "DogeCV")
public class FocalLengthCalculator extends LinearOpMode {
    private Dogeforia vuforia;
    private final String VUFORIA_KEY = "AQRacK7/////AAAAGea1bsBsYEJvq6S3KuXK4PYTz4IZmGA7SV88bdM7l26beSEWkZTUb8H352Bo/ZMC6krwmfEuXiK7d7qdFkeBt8BaD0TZAYBMwHoBkb7IBgMuDF4fnx2KiQPOvwBdsIYSIFjiJgGlSj8pKZI+M5qiLb3DG3Ty884EmsqWQY0gjd6RNhtSR+6oiXazLhezm9msyHWZtX5hQFd9XoG5npm4HoGaZNdB3g5YCAQNHipjTm3Vkf71rG/Fffif8UTCI1frmKYtb4RvqiixDSPrD6OG6YmbsPOYUt2RZ6sSTreMzVL76CNfBTzmpo2V0E6KKP2y9N19hAum3GZu3G/1GEB5D+ckL/CXk4JM66sJw3PGucCs";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;

    private final double GOLD_WIDTH_IN = 2;
    private ThunderGoldAlignDetector detector;

    @Override
    public void runOpMode() {
        detector = new ThunderGoldAlignDetector();


        Dogeforia.Parameters parameters = new Dogeforia.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = CAMERA_CHOICE;

        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);

        vuforia.enableConvertFrameToBitmap();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, true);
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
        CameraDevice.getInstance().setFlashTorchMode(true);

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        //vuforia.showDebug(); //Don't enable this since it causes a crash
        vuforia.start();

        waitForStart();

        while (opModeIsActive()) {
            double perpendicularDistance = 12;
            if (detector.isFound() && !Double.isInfinite(perpendicularDistance) && !detector.bestRectIsNull()) {
                telemetry.addData("Focal Length", calculateFocalLength(detector.getBestRectWidth(), perpendicularDistance));
                telemetry.update();
            }
        }

        detector.disable();
    }

    private double calculateFocalLength(int goldWithPX, double distanceFromObjIn) {
        return goldWithPX * distanceFromObjIn / GOLD_WIDTH_IN;
    }
}
