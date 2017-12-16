// code by jph
package ch.ethz.idsc.retina.gui.gokart.top;

import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.util.List;
import java.util.Objects;

import ch.ethz.idsc.owl.data.Stopwatch;
import ch.ethz.idsc.owl.gui.win.GeometricLayer;
import ch.ethz.idsc.retina.alg.slam.Se2MultiresSamples;
import ch.ethz.idsc.retina.util.GlobalAssert;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;

public class SlamDunk {
  // private final BufferedImage bufferedImage;
  private final byte[] bytes;
  private final int WIDTH;
  private Se2MultiresSamples se2MultiresSamples;

  public SlamDunk(BufferedImage bufferedImage) {
    // this.bufferedImage = bufferedImage;
    WIDTH = bufferedImage.getWidth();
    // HEIGHT = bufferedImage.getHeight();
    WritableRaster writableRaster = bufferedImage.getRaster();
    DataBufferByte dataBufferByte = (DataBufferByte) writableRaster.getDataBuffer();
    bytes = dataBufferByte.getData();
  }

  public void set(Se2MultiresSamples se2MultiresSamples) {
    this.se2MultiresSamples = se2MultiresSamples;
  }

  public Tensor fit(GeometricLayer geometricLayer, List<Tensor> list) {
    Stopwatch stopwatch = Stopwatch.started();
    Tensor result = IdentityMatrix.of(3);
    int pushed = 0;
    for (int level = 0; level < se2MultiresSamples.levels(); ++level) {
      int cmp = -1;
      Tensor best = null;
      for (Tensor delta : se2MultiresSamples.level(level)) {
        geometricLayer.pushMatrix(delta);
        int eval = 0;
        for (Tensor pnts : list)
          for (Tensor x : pnts) {
            Point2D point2D = geometricLayer.toPoint2D(x);
            eval += evaluate(point2D);
          }
        GlobalAssert.that(0 <= eval);
        if (cmp < eval) {
          best = delta;
          cmp = eval;
        }
        geometricLayer.popMatrix();
      }
      if (Objects.nonNull(best)) {
        geometricLayer.pushMatrix(best);
        result = result.dot(best);
        ++pushed;
      } else
        throw new RuntimeException("ARG"); // FIXME
    }
    for (int count = 0; count < pushed; ++count)
      geometricLayer.popMatrix();
    double duration = stopwatch.display_seconds();
    System.out.println(duration + "[s]");
    return result;
  }

  private int evaluate(Point2D point2D) {
    int sum = 0;
    // int x = (int) Math.round(point2D.getX()); // TODO not optimal, why not (int)
    int x = (int) point2D.getX(); // TODO not optimal, why not (int)
    if (0 <= x && x < WIDTH) {
      // int y = (int) Math.round(point2D.getY());
      int y = (int) point2D.getY();
      if (0 <= y && y < WIDTH)
        sum += bytes[x + WIDTH * y] & 0xff;
    }
    return sum;
  }
}
