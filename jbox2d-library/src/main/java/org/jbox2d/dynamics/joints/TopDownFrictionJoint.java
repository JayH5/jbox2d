package org.jbox2d.dynamics.joints;

import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.SolverData;
import org.jbox2d.pooling.IWorldPool;

/**
 * Friction joint for top-down scenes. Simulates friction between the ground and an object.
 * Original C++ source: https://github.com/mikelikespie/box2d-demo/blob/car/Source/Dynamics/Joints/b2FrictionJoint.cpp
 *
 * Created by jamie on 2014/09/15.
 */
public class TopDownFrictionJoint extends Joint {

  private static final float ACCELERATION_GRAVITY = 9.81f;

  private final float m_kineticCOF;
  private final float m_frictionTorque;

  private final Vec2 m_lambda_p = new Vec2();
  private float m_lambda_a_p = 0f;

  private int m_indexA;

  private float m_massA;
  private float m_invMassA;

  private float m_IA;
  private float m_invIA;

  private float m_kineticFriction;

  private float m_dt;
  private float m_inv_dt;

  protected TopDownFrictionJoint(IWorldPool worldPool, TopDownFrictionJointDef def) {
    super(worldPool, def);
    m_kineticCOF = def.kineticCOF;
    m_frictionTorque = def.frictionTorque;

    m_bodyA = def.bodyA;
    m_bodyB = null;
  }

  @Override
  public void getAnchorA(Vec2 vec2) {
    vec2.set(m_bodyA.getWorldCenter());
  }

  @Override
  public void getAnchorB(Vec2 vec2) {
    vec2.setZero();
  }

  @Override
  public void getReactionForce(float inv_dt, Vec2 argOut) {
    argOut.set(m_lambda_p).mulLocal(inv_dt);
  }

  @Override
  public float getReactionTorque(float inv_dt) {
    return inv_dt * m_lambda_a_p;
  }

  @Override
  public void initVelocityConstraints(SolverData data) {
    m_indexA = m_bodyA.m_islandIndex;

    // Mass
    m_massA = m_bodyA.m_mass;
    m_invMassA = m_bodyA.m_invMass;

    // Moment of inertia
    m_IA = m_bodyA.m_I;
    m_invIA = m_bodyA.m_invI;

    // Calculate kinetic friction due to gravity
    m_kineticFriction = m_massA * ACCELERATION_GRAVITY * m_kineticCOF;

    // Calculate time step per iteration
    m_dt = data.step.dt / data.step.velocityIterations;
    m_inv_dt = data.step.inv_dt * data.step.velocityIterations;

    m_lambda_p.setZero();
  }

  @Override
  public void solveVelocityConstraints(SolverData data) {
    if (m_kineticFriction > 0f) {
      final Vec2 linearVelocity = data.velocities[m_indexA].v;

      // Calculate current momentum and force
      final Vec2 momentum = pool.popVec2();
      final Vec2 force = pool.popVec2();
      momentum.set(linearVelocity).mulLocal(m_massA);
      force.set(momentum).mulLocal(m_inv_dt);

      // Clamp the force to the kinetic friction limit
      float forceMagnitude = force.length();
      if (forceMagnitude > m_kineticFriction) {
        force.mulLocal(m_kineticFriction / forceMagnitude);
      }

      // Calculate the new momentum from the force
      momentum.set(force).mulLocal(m_dt);

      // Update the total momentum
      m_lambda_p.addLocal(momentum);

      // Calculate the velocity change due to friction
      final Vec2 frictionVelocity = pool.popVec2();
      frictionVelocity.set(momentum).mulLocal(m_invMassA);

      // Counteract the velocity
      data.velocities[m_indexA].v.subLocal(frictionVelocity);

      // Give back the vectors to the pool
      pool.pushVec2(3);
    }

    if (m_frictionTorque != 0f) {
      final float angularVelocity = data.velocities[m_indexA].w;

      // Calculate angular momentum and torque
      float angularMomentum = m_IA * angularVelocity;
      float torque = m_inv_dt * angularMomentum;

      // Clamp the torque
      torque = MathUtils.clamp(torque, -m_frictionTorque, m_frictionTorque);

      // Calculate the new angular momentum from the torque
      angularMomentum = m_dt * torque;

      // Update the total momentum
      m_lambda_a_p += angularMomentum;

      // Calculate the angular velocity change due to friction
      float frictionAngularVelocity = angularMomentum * m_invIA;

      // Counteract the angular velocity
      data.velocities[m_indexA].w -= frictionAngularVelocity;
    }
  }

  @Override
  public boolean solvePositionConstraints(SolverData solverData) {
    return true;
  }
}
