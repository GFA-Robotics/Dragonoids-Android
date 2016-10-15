package com.qualcomm.ftcrobotcontroller.opmodes;

public class DragonoidsAutoRedDelay extends DragonoidsAuto {
    public final Alliance alliance = Alliance.Red;
    public final int waitTime = 15000; // Milliseconds to wait before starting autonomous
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            sleep(waitTime);
            super.autonomous(alliance);
        }
        catch (Exception e) {
            DragonoidsGlobal.stopAll();
            throw e;
        }
    }
}
