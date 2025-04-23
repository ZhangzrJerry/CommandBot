package frc.robot.utils.logging;

import edu.wpi.first.math.Matrix;
import org.littletonrobotics.junction.Logger;

public class LoggedUtil {
  public void logMatrix(String key, double[][] matrix) {
    if (matrix.length == 0 || matrix[0].length == 0) {
      return;
    }
    for (int i = 0; i < matrix.length; i++) {
      for (int j = 0; j < matrix[i].length; j++) {
        Logger.recordOutput(key + "/" + i + "/" + j, matrix[i][j]);
      }
    }
  }

  public static void logMatrix(String key, Matrix<?, ?> matrix) {
    if (matrix.get(0, 0) == 0) {
      return;
    }
    for (int i = 0; i < matrix.getNumRows(); i++) {
      for (int j = 0; j < matrix.getNumCols(); j++) {
        Logger.recordOutput(key + "/" + i + "/" + j, matrix.get(i, j));
      }
    }
  }

  private LoggedUtil() {}
}
