// code by jph, gjoel
package ch.ethz.idsc.retina.lidar;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;

/** events generated by lidar */
public class LidarPolarEvent {
  /** timestamp of event in [us] */
  public final int usec;
  /** spacial coordinates */
  public final float[] coords;
  /** intensity of reflection [0, 1, ..., 255] 255 == most intensive return */
  public final byte intensity;

  public LidarPolarEvent(int usec, float[] coords, byte intensity) {
    this.usec = usec;
    this.coords = coords;
    this.intensity = intensity;
  }

  public Scalar getAzimuth() {
    return RealScalar.of(coords[0]);
  }

  public Scalar getElevation() {
    return RealScalar.of(coords[1]);
  }

  public Scalar getDistance() {
    return RealScalar.of(coords[2]);
  }
}