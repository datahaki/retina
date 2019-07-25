// code by ynager
package ch.ethz.idsc.gokart.core.plan;

import ch.ethz.idsc.gokart.core.map.AbstractMapping;
import ch.ethz.idsc.gokart.core.map.GenericBayesianMapping;
import ch.ethz.idsc.gokart.core.map.ImageGrid;
import ch.ethz.idsc.gokart.core.map.SightLinesMapping;
import ch.ethz.idsc.gokart.core.slam.PredefinedMap;
import ch.ethz.idsc.gokart.gui.trj.Se2UniformResample;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.retina.util.sys.AppResources;
import ch.ethz.idsc.sophus.crv.subdiv.CurveSubdivision;
import ch.ethz.idsc.tensor.RationalScalar;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Degree;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Ramp;

public class TrajectoryConfig {
  public static final TrajectoryConfig GLOBAL = AppResources.load(new TrajectoryConfig());
  /***************************************************/
  public Scalar planningPeriod = Quantity.of(1, SI.SECOND); // 1[s] == 1[Hz]
  public Scalar expandFraction = RationalScalar.of(3, 4);
  public Scalar planningOffset = Quantity.of(2.5, SI.METER);
  /** horizonDistance is unit-less because it entails all three: x, y, heading using Se2Wrap
   * post 20180904: changed horizonDistance from 8 to 10 so that the gokart plans through a gateway
   * post 20181025: changed horizonDistance to 12 */
  public Scalar horizonDistance = RealScalar.of(10);
  /** number of different steering angles for path planning
   * value has to be an integer */
  public Scalar controlResolution = RealScalar.of(9);
  /** rotation per meter driven is at least 23[deg/m]
   * 20180429_minimum_turning_radius.pdf
   * 20180517: reduced value to 20[deg/m] to be more conservative and avoid extreme steering
   * 20181025: reduced value to 15[deg/m] */
  public Scalar maxRotation = Quantity.of(15, "deg*m^-1");
  /** half angle of conic goal region */
  public Scalar coneHalfAngle = Degree.of(18);
  public Tensor goalRadiusFactor = Tensors.vector(4, 4, 2);
  /** true = SightLinesMapping
   * false = GenericBayesianMapping */
  public Boolean mapSightLines = true;
  /** preferred waypoint spacing */
  public Scalar waypointsSpacing = Quantity.of(2.5, SI.METER);

  /***************************************************/
  /** @param tangentSpeed with unit "m*s^-1"
   * @return non-negative */
  public Scalar getCutoffDistance(Scalar tangentSpeed) {
    return Ramp.FUNCTION.apply(tangentSpeed) //
        .multiply(planningPeriod) // for instance 1[s]
        .add(planningOffset); // for instance 2.5[m]
  }

  public Scalar expandTimeLimit() {
    return planningPeriod.multiply(expandFraction);
  }

  /** @return */
  public static PredefinedMap getPredefinedMapObstacles() {
    return PredefinedMap.DUBILAB_OBSTACLES_20190314;
  }

  public AbstractMapping<? extends ImageGrid> getAbstractMapping() {
    return mapSightLines //
        ? SightLinesMapping.defaultObstacle()
        : GenericBayesianMapping.createObstacleMapping();
  }

  public Tensor resampledWaypoints(Tensor curve) {
    return resampledWaypoints(curve, true);
  }

  public Tensor resampledWaypoints(Tensor curve, boolean cyclic) {
    CurveSubdivision curveSubdivision = Se2UniformResample.of(waypointsSpacing);
    return cyclic ? curveSubdivision.cyclic(curve) : curveSubdivision.string(curve);
  }
}