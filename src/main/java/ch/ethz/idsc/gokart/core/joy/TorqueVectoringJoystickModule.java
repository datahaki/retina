// code by mh
package ch.ethz.idsc.gokart.core.joy;

import java.util.Objects;
import java.util.Optional;

import ch.ethz.idsc.gokart.core.fuse.DavisImuTracker;
import ch.ethz.idsc.gokart.core.fuse.Vlp16PassiveSlowing;
import ch.ethz.idsc.gokart.dev.rimo.RimoGetEvent;
import ch.ethz.idsc.gokart.dev.rimo.RimoGetListener;
import ch.ethz.idsc.gokart.dev.rimo.RimoPutEvent;
import ch.ethz.idsc.gokart.dev.rimo.RimoPutHelper;
import ch.ethz.idsc.gokart.dev.rimo.RimoSocket;
import ch.ethz.idsc.gokart.dev.steer.SteerColumnInterface;
import ch.ethz.idsc.gokart.dev.steer.SteerConfig;
import ch.ethz.idsc.gokart.dev.steer.SteerMapping;
import ch.ethz.idsc.gokart.gui.top.ChassisGeometry;
import ch.ethz.idsc.retina.dev.joystick.ManualControlInterface;
import ch.ethz.idsc.retina.util.math.Magnitude;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.retina.util.sys.ModuleAuto;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.Differences;
import ch.ethz.idsc.tensor.qty.Quantity;
import ch.ethz.idsc.tensor.sca.Tan;

/** abstract base class for all torque vectoring modules:
 * 
 * {@link SimpleTorqueVectoringJoystickModule}
 * {@link ImprovedTorqueVectoringJoystickModule}
 * {@link ImprovedNormalizedTorqueVectoringJoystickModule} */
abstract class TorqueVectoringJoystickModule extends GuideJoystickModule<RimoPutEvent> //
    implements RimoGetListener {
  private final SteerMapping steerMapping = SteerConfig.GLOBAL.getSteerMapping();
  private final TorqueVectoringInterface torqueVectoringInterface;
  private final Vlp16PassiveSlowing vlp16PassiveSlowing;
  // ---
  private Scalar meanTangentSpeed = Quantity.of(0, SI.VELOCITY);

  TorqueVectoringJoystickModule(TorqueVectoringInterface torqueVectoring) {
    this.torqueVectoringInterface = torqueVectoring;
    vlp16PassiveSlowing = ModuleAuto.INSTANCE.getInstance(Vlp16PassiveSlowing.class);
  }

  @Override // from AbstractModule
  final void protected_first() {
    RimoSocket.INSTANCE.addPutProvider(this);
    RimoSocket.INSTANCE.addGetListener(this);
  }

  @Override // from AbstractModule
  final void protected_last() {
    RimoSocket.INSTANCE.removePutProvider(this);
    RimoSocket.INSTANCE.removeGetListener(this);
  }

  /***************************************************/
  @Override // from GuideJoystickModule
  final Optional<RimoPutEvent> control( //
      SteerColumnInterface steerColumnInterface, ManualControlInterface joystick) {
    Scalar theta = steerMapping.getAngleFromSCE(steerColumnInterface); // steering angle of imaginary front wheel
    Scalar rotationPerMeterDriven = Tan.FUNCTION.apply(theta).divide(ChassisGeometry.GLOBAL.xAxleRtoF); // m^-1
    // why isn't theta rad/m?
    Scalar power = // labjackAdcLcmClient.getAheadSigned();
        // System.out.println("get ahead " + power);
        Differences.of(joystick.getAheadPair_Unit()).Get(0); // unitless in the interval [-1, 1]
    // compute wanted motor torques / no-slip behavior (sorry Jan for corrective factor)
    Scalar wantedRotationRate = rotationPerMeterDriven.multiply(meanTangentSpeed); // unit s^-1
    // compute (negative) angular slip
    Scalar gyroZ = DavisImuTracker.INSTANCE.getGyroZ(); // unit s^-1
    Scalar angularSlip = wantedRotationRate.subtract(gyroZ);
    // ---
    Tensor powers = torqueVectoringInterface.powers( //
        rotationPerMeterDriven, meanTangentSpeed, angularSlip, power, gyroZ);
    Tensor torquesARMS = powers.multiply(ManualConfig.GLOBAL.torqueLimit); // vector of length 2
    // ---
    short arms_rawL = Magnitude.ARMS.toShort(torquesARMS.Get(0));
    short arms_rawR = Magnitude.ARMS.toShort(torquesARMS.Get(1));
    // System.out.println("arms_rawl: " + arms_rawL + " arms_rawr " + arms_rawR);
    return Optional.of(RimoPutHelper.operationTorque( //
        (short) -arms_rawL, // sign left invert
        (short) +arms_rawR // sign right id
    ));
  }

  @Override // from RimoGetListener
  public final void getEvent(RimoGetEvent getEvent) {
    if (Objects.nonNull(vlp16PassiveSlowing))
      vlp16PassiveSlowing.bypassSafety();
    meanTangentSpeed = ChassisGeometry.GLOBAL.odometryTangentSpeed(getEvent);
  }
}
