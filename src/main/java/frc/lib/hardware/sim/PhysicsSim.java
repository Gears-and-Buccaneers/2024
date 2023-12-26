package frc.lib.hardware.sim;

import java.util.ArrayList;

public class PhysicsSim {
    private static final PhysicsSim sim = new PhysicsSim();

    public static PhysicsSim getInstance() {
        return sim;
    }
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();
        }
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();
}
