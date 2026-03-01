import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.helpers.MythicalMath;

class AimingTest {
    
    @BeforeEach // this method will run before each test
  void setup() {
  }


  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    //m_intake.close(); // destroy our intake object
  }

  @Test // marks this method as a test
  void TenTenXY() {
    var Rot = MythicalMath.aimProjectileAtPoint(new Translation3d(0, 0, 0), new Translation3d(10, 10, 0), 20, new Translation3d(0, 0, 0));
    System.out.println(Rot.get_0());
    assertEquals(
        Math.toDegrees(1.425), Rot.get_0(), 2);
  }

  @Test
  void FiveFiveFiveXYZ() {
    var Rot = MythicalMath.aimProjectileAtPoint(new Translation3d(0, 0, 0), new Translation3d(5, 5, 5), 20, new Translation3d(0, 0, 0));
    System.out.println(Rot.get_0());
    System.out.println(Rot.get_1());
    assertEquals(
        Math.toDegrees(1.477), Rot.get_0(), 2);
    assertEquals(
        Math.toDegrees(0.785), Rot.get_1(), 2);
  }
}