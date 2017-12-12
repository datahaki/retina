// code by jph
package ch.ethz.idsc.retina.gui.gokart.top;

import java.awt.Graphics2D;

import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.owl.gui.win.GeometricLayer;
import ch.ethz.idsc.owl.math.map.Se2Utils;
import ch.ethz.idsc.retina.dev.zhkart.pos.GokartPoseInterface;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.qty.QuantityMagnitude;
import ch.ethz.idsc.tensor.sca.ScalarUnaryOperator;

public abstract class AbstractGokartRender implements RenderInterface {
  private static final ScalarUnaryOperator TO_METER = QuantityMagnitude.singleton("m");
  // ---
  private final GokartPoseInterface gokartPoseInterface;

  public AbstractGokartRender(GokartPoseInterface gokartPoseInterface) {
    this.gokartPoseInterface = gokartPoseInterface;
  }

  @Override
  public final void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Tensor state = gokartPoseInterface.getPose(); // units {x[m], y[m], angle[]}
    Scalar x = TO_METER.apply(state.Get(0));
    Scalar y = TO_METER.apply(state.Get(1));
    Scalar angle = state.Get(2);
    geometricLayer.pushMatrix(Se2Utils.toSE2Matrix(Tensors.of(x, y, angle)));
    protected_render(geometricLayer, graphics);
    geometricLayer.popMatrix();
  }

  public abstract void protected_render(GeometricLayer geometricLayer, Graphics2D graphics);
}
