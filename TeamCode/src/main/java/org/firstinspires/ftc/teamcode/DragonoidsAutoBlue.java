package com.qualcomm.ftcrobotcontroller.opmodes;

public class DragonoidsAutoBlue extends DragonoidsAuto {
    public final Alliance alliance = Alliance.Blue;
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
