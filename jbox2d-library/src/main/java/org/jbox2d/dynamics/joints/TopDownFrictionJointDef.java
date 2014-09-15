package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;

/**
 * Top down friction joint definition. This requires a single body.
 * Created by jamie on 2014/09/15.
 */
public class TopDownFrictionJointDef extends JointDef {

  float kineticCOF;

  float frictionTorque;

  public TopDownFrictionJointDef() {
    super(JointType.TOPDOWNFRICTION);
  }

  public void initialize(Body bodyA, float kineticCOF, float frictionTorque) {
    this.bodyA = bodyA;
    this.kineticCOF = kineticCOF;
    this.frictionTorque = frictionTorque;

    this.bodyB = null;
    this.collideConnected = false;
  }
}
