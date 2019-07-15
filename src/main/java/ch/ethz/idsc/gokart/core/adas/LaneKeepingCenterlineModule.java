// code by am
package ch.ethz.idsc.gokart.core.adas;

import java.util.Optional;

import ch.ethz.idsc.gokart.calib.steer.SteerMapping;
import ch.ethz.idsc.gokart.core.pos.GokartPoseEvent;
import ch.ethz.idsc.gokart.core.pos.GokartPoseEvents;
import ch.ethz.idsc.gokart.core.pos.GokartPoseLcmClient;
import ch.ethz.idsc.gokart.core.pos.GokartPoseListener;
import ch.ethz.idsc.gokart.core.pure.ClothoidPlan;
import ch.ethz.idsc.gokart.core.pure.ClothoidPursuitConfig;
import ch.ethz.idsc.gokart.core.pure.CurveClothoidPursuitPlanner;
import ch.ethz.idsc.gokart.core.pure.CurveSe2PursuitLcmClient;
import ch.ethz.idsc.gokart.core.slam.LocalizationConfig;
import ch.ethz.idsc.gokart.dev.steer.SteerConfig;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.retina.util.sys.AbstractClockedModule;
import ch.ethz.idsc.sophus.lie.se2.Se2GroupElement;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.ref.TensorListener;
import ch.ethz.idsc.tensor.sca.Clip;
import ch.ethz.idsc.tensor.sca.Clips;

/**  */
public class LaneKeepingCenterlineModule extends AbstractClockedModule implements //
    GokartPoseListener, TensorListener {
  private static final Tensor OFS_L = Tensors.fromString("{0, +1[m], 0}").unmodifiable();
  private static final Tensor OFS_R = Tensors.fromString("{0, -1[m], 0}").unmodifiable();
  private static final Scalar PERIOD = Quantity.of(0.1, SI.SECOND);
  // ---
  private final CurveSe2PursuitLcmClient curveSe2PursuitLcmClient = new CurveSe2PursuitLcmClient();
  private final GokartPoseLcmClient gokartPoseLcmClient = new GokartPoseLcmClient();
  private final SteerMapping steerMapping = SteerConfig.GLOBAL.getSteerMapping();
  // ---
  private GokartPoseEvent gokartPoseEvent = GokartPoseEvents.motionlessUninitialized();
  private Optional<Tensor> optionalCurve = Optional.empty();
  public Optional<Clip> optionalPermittedRange;
  public Tensor velocity = GokartPoseEvents.motionlessUninitialized().getVelocity();
  private Tensor laneBoundaryL;
  private Tensor laneBoundaryR;

  @Override // from AbstractModule
  public void first() {
    gokartPoseLcmClient.addListener(this);
    gokartPoseLcmClient.startSubscriptions();
    curveSe2PursuitLcmClient.addListener(this);
    curveSe2PursuitLcmClient.startSubscriptions();
  }

  @Override // from AbstractModule
  public void last() {
    gokartPoseLcmClient.stopSubscriptions();
    curveSe2PursuitLcmClient.stopSubscriptions();
  }

  @Override // from AbstractClockedModule
  protected void runAlgo() {
    boolean isQualityOk = LocalizationConfig.GLOBAL.isQualityOk(gokartPoseEvent);
    Tensor pose = isQualityOk //
        ? gokartPoseEvent.getPose() //
        : GokartPoseEvents.motionlessUninitialized().getPose();
    velocity = isQualityOk //
        ? gokartPoseEvent.getVelocity() //
        : GokartPoseEvents.motionlessUninitialized().getVelocity();
    boolean isPresent = optionalCurve.isPresent();
    Tensor curve = isPresent //
        ? optionalCurve.get()//
        : null;
    if (isPresent && isQualityOk) {
      optionalPermittedRange = getPermittedRange(curve, pose);
      System.out.println(optionalPermittedRange);
    }
  }

  @Override // from AbstractClockedModule
  protected Scalar getPeriod() {
    return PERIOD;
  }

  @Override // from GokartPoseListener
  public void getEvent(GokartPoseEvent gokartPoseEvent) {
    this.gokartPoseEvent = gokartPoseEvent;
  }

  public void setCurve(Optional<Tensor> optional) {
    if (optional.isPresent()) {
      optionalCurve = optional;
      laneBoundaryL = Tensor.of(optional.get().stream() //
          .map(Se2GroupElement::new) //
          .map(se2GroupElement -> se2GroupElement.combine(OFS_L)));
      laneBoundaryR = Tensor.of(optional.get().stream() //
          .map(Se2GroupElement::new) //
          .map(se2GroupElement -> se2GroupElement.combine(OFS_R)));
    } else {
      System.err.println("Curve missing");
      optionalCurve = Optional.empty();
    }
  }

  final Optional<Tensor> getCurve() {
    System.out.println("got curve");
    return optionalCurve;
  }

  public Optional<Clip> getPermittedRange(Tensor curve, Tensor pose) {
    Scalar steerlimitLSCE = Quantity.of(0, "SCE");
    Scalar steerlimitRSCE = Quantity.of(0, "SCE");
    if (1 < curve.length()) {
      System.out.println("ifloop entered :)");
      ClothoidPursuitConfig clothoidPursuitConfig = new ClothoidPursuitConfig();
      // large value is a hack to get a solution
      clothoidPursuitConfig.turningRatioMax = Quantity.of(1000, SI.PER_METER);
      Optional<ClothoidPlan> optionalL = //
          new CurveClothoidPursuitPlanner(clothoidPursuitConfig).getPlan(pose, Quantity.of(0, SI.VELOCITY), laneBoundaryL, true);
      Optional<ClothoidPlan> optionalR = //
          new CurveClothoidPursuitPlanner(clothoidPursuitConfig).getPlan(pose, Quantity.of(0, SI.VELOCITY), laneBoundaryR, true);
      System.out.println(optionalL);
      if (optionalL.isPresent()) {
        Scalar steerlimitLratio = optionalL.get().ratio();
        steerlimitLSCE = steerMapping.getSCEfromRatio(steerlimitLratio);
        System.out.println("Limit L: " + steerlimitLSCE);
      }
      if (optionalR.isPresent()) {
        Scalar steerlimitRratio = optionalR.get().ratio();
        steerlimitRSCE = steerMapping.getSCEfromRatio(steerlimitRratio);
        System.out.println("Limit R: " + steerlimitRSCE);
      }
      return Optional.of(Clips.interval(steerlimitRSCE, steerlimitLSCE));
    }
    System.out.println("no steer limit found");
    System.out.println("ifloop not entered :(");
    return Optional.empty();
  }

  @Override // from TensorListener
  public void tensorReceived(Tensor tensor) {
    setCurve(tensor.length() == 0 //
        ? Optional.empty()
        : Optional.of(tensor));
  }
}
