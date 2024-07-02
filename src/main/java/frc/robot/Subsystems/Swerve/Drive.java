package frc.robot.Subsystems.Swerve;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.TeleopDriveController;

public class Drive extends SubsystemBase {

      private final Module[] modules = new Module[4];
      TeleopDriveController teleopDriveController;
      private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
     public static final Lock odometryLock = new ReentrantLock();
            
      private final GyroIO gyroIO;
      private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
      private boolean brakeModeEnabled = true;
  


    public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br){
    this.gyroIO = gyroIO;
    modules[0] = new Module(fl,0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);  

    teleopDriveController = new TeleopDriveController();
    setBrakeMode(brakeModeEnabled);
    }

    @Override
    public void periodic(){
        ChassisSpeeds teleopSpeeds = teleopDriveController.update();
        gyroIO.updateInputs(gyroInputs);

            // Plain teleop drive
            desiredSpeeds = teleopSpeeds;
            // Add auto aim if present
    }

    public void acceptTeleopInput(double controllerX, double controllerY, double controllerOmega, boolean robotRelative){
        if (DriverStation.isTeleopEnabled()) {

            teleopDriveController.acceptDriveInput(
                controllerX, controllerY, controllerOmega, robotRelative);
          }
        }

    public void stop(){
        for (var module : modules) {
            module.stop();
          }
    }

      private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }


}
