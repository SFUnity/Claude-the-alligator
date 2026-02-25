import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class encoderReading {
  Turret turret;

  @BeforeEach
  public void setUp() {
    // Setup code here
    turret = new Turret(new TurretIOSim());
  }

  @AfterEach
  public void tearDown() {
    // Teardown code here
  }

  @Test
  public void testEncoderReading() {
    turret.getMotorOffsetDegsTestable(0.1, 0);
  }
}
