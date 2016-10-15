package com.qualcomm.ftcrobotcontroller.opmodes;

public class DragonoidsAutoRed extends DragonoidsAuto {
    public final Alliance alliance = Alliance.Red;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.autonomous(alliance);
        }
        catch (Exception e) {
            DragonoidsGlobal.stopAll();
            throw e;
        }
    }
}
