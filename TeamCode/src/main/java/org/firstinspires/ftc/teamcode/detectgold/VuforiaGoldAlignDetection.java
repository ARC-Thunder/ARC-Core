package org.firstinspires.ftc.teamcode.detectgold;

import com.disnodeteam.dogecv.Dogeforia;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class VuforiaGoldAlignDetection extends GoldAlignDetection {
    private Dogeforia vuforia;

    public VuforiaGoldAlignDetection(HardwareMap hardwareMap, String VUFORIA_KEY, VuforiaLocalizer.CameraDirection CAMERA_CHOICE, int cameraIndex) {
        super(hardwareMap, cameraIndex, true);

        Dogeforia.Parameters parameters = new Dogeforia.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.start();
    }


    public void stopVuforia() {
        vuforia.stop();
    }

    public Dogeforia getVuforia() {
        return vuforia;
    }
}
