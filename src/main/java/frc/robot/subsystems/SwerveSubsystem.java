package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {

// Intializing Swerve Object
public SwerveDrive m_swerveDrive;

// Max Speed
double maximumSpeed = Units.feetToMeters(4.5);



    public SwerveSubsystem(File m_directory){
          
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try{
          this.m_swerveDrive = new SwerveParser(m_directory).createSwerveDrive(maximumSpeed);

        } catch (Exception error){
          throw new RuntimeException(error);
        }
        this.m_swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        this.m_swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        //setupPathPlanner();
      }


      /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(double translationX, double translationY, double angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      this.m_swerveDrive.drive(new Translation2d(translationX * this.m_swerveDrive.getMaximumVelocity(),
                                          translationY * this.m_swerveDrive.getMaximumVelocity()),
                        angularRotationX * this.m_swerveDrive.getMaximumAngularVelocity(),
                        false,
                        false);
    });
  }


      public void robotBrake() {
        this.m_swerveDrive.lockPose();
      }

  

    
  



}


