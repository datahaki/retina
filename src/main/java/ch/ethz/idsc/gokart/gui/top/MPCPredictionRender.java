// code by mh
package ch.ethz.idsc.gokart.gui.top;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Objects;

import ch.ethz.idsc.gokart.core.mpc.ControlAndPredictionSteps;
import ch.ethz.idsc.gokart.core.mpc.MPCControlUpdateListener;
import ch.ethz.idsc.gokart.core.pos.GokartPoseHelper;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.win.GeometricLayer;
import ch.ethz.idsc.retina.util.math.SI;
import ch.ethz.idsc.sophus.group.Se2Utils;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.Quantity;

public class MPCPredictionRender implements MPCControlUpdateListener, RenderInterface {
  // TODO JPH/MH the units of scale are ignored -> remove unit of scale
  private static final Scalar SCALE = Quantity.of(0.3, SI.METER);
  // ---
  private ControlAndPredictionSteps _controlAndPredictionSteps;

  @Override // from MPCControlUpdateInterface
  public void getControlAndPredictionSteps(ControlAndPredictionSteps controlAndPredictionSteps) {
    this._controlAndPredictionSteps = controlAndPredictionSteps;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    ControlAndPredictionSteps controlAndPredictionSteps = _controlAndPredictionSteps;
    if (Objects.nonNull(controlAndPredictionSteps)) {
      Tensor positions = controlAndPredictionSteps.toPositions();
      graphics.setColor(Color.GREEN);
      graphics.draw(geometricLayer.toPath2D(positions)); // draw positions as path
      // acceleration visualization
      Tensor accelerations = controlAndPredictionSteps.toAccelerations();
      Tensor poses = controlAndPredictionSteps.toXYA();
      for (int index = 0; index < accelerations.length(); ++index) {
        Tensor pose = GokartPoseHelper.toUnitless(poses.get(index));
        geometricLayer.pushMatrix(Se2Utils.toSE2Matrix(pose));
        Color color = Scalars.lessThan(accelerations.Get(index), Quantity.of(0, SI.ACCELERATION)) //
            ? Color.RED
            : Color.GREEN;
        graphics.setColor(color);
        Scalar acc = accelerations.Get(index);
        // TODO JPH/MH use Magnitude.ACC.toDouble...
        Tensor start = Tensors.vector(-acc.number().doubleValue() * 0.8, acc.number().doubleValue());
        Tensor mid = Tensors.vector(0, 0);
        Tensor end = Tensors.vector(-acc.number().doubleValue() * 0.8, -acc.number().doubleValue());
        start = start.multiply(SCALE);
        mid = mid.multiply(SCALE);
        end = end.multiply(SCALE);
        graphics.draw(geometricLayer.toPath2D(Tensors.of(start, mid, end)));
        geometricLayer.popMatrix();
      }
    }
  }
}
