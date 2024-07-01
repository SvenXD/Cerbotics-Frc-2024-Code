package frc.robot.Subsystems.Swerve;

public class drive {


    GyroIO gyroIO;
    ModuleIO moduleIO;
    Module[] modules;


    public drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br){
        this.gyroIO = gyroIO;

        modules[0] = new Module(fl, 0);
        modules[1] = new Module(fr, 1);
        modules[2] = new Module(bl, 2);
        modules[3] = new Module(br, 3);
        lastMovementTimer.start();
        setBrakeMode(true);  
    }

     /*/   public void go(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        }*/
}
