import math
import numpy as np  
from acsl_pychrono.control.outerloop_safetymech import OuterLoopSafetyMechanism
from acsl_pychrono.control.MRAC.mrac_gains import MRACGains
from acsl_pychrono.simulation.ode_input import OdeInput
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.control.control import Control
from acsl_pychrono.control.base_mrac import BaseMRAC
from acsl_pychrono.control.MRAC.m_mrac import M_MRAC
from acsl_pychrono.control.projection_operator import ProjectionOperator

class MRAC(BaseMRAC, Control):
  def __init__(self, gains: MRACGains, ode_input: OdeInput, flight_params: FlightParams, timestep: float):
    super().__init__(odein=ode_input, gains=gains)
    self.gains = gains
    self.fp = flight_params
    self.timestep = timestep
    self.safety_mechanism = OuterLoopSafetyMechanism(gains, self.fp.G_acc)
    self.dy = np.zeros((self.gains.number_of_states, 1))
    # Initial conditions
    self.y = np.zeros((self.gains.number_of_states, 1))

  def computeControlAlgorithm(self, ode_input: OdeInput):
    """
    Compute all intermediate variables and control inputs once per RK4 step to compute the dy for RK4.
    """
    # Update the vehicle state and user-defined trajectory
    self.odein = ode_input
    
    # ODE state            
    self.state_phi_ref_diff = self.y[0:2] # State of the differentiator for phi_ref (roll_ref)
    self.state_theta_ref_diff = self.y[2:4] # State of the differentiator for theta_ref (pitch_ref)
    self.x_ref_tran = self.y[4:10] # Reference model state
    self.integral_position_tracking_ref = self.y[10:13] # Integral of ('translational_position_in_I_ref' - 'translational_position_in_I_user')
    self.K_hat_x_tran = self.y[13:31] # \hat{K}_x (translational)
    self.K_hat_r_tran = self.y[31:40] # \hat{K}_r (translational)
    self.Theta_hat_tran = self.y[40:58] # \hat{\Theta} (translational)
    self.omega_ref = self.y[58:61] # Reference model rotational dynamics
    self.K_hat_x_rot = self.y[61:70] # \hat{K}_x (rotational)
    self.K_hat_r_rot = self.y[70:79] # \hat{K}_r (rotational)
    self.Theta_hat_rot = self.y[79:97] # \hat{\Theta} (rotational)
    self.integral_e_rot = self.y[97:100] # Integral of 'e_rot' = (angular_velocity - omega_ref) 
    self.integral_angular_error = self.y[100:103] # Integral of angular_error = attitude - attitude_ref
    self.integral_e_omega_ref_cmd = self.y[103:106] #Integral of (omega_ref - omega_cmd)

    # Reshapes all adaptive gains to their correct (row, col) shape as matrices
    self.reshapeAdaptiveGainsToMatricesMRAC()

    # compute translational and rotational trajectory tracking error
    self.computeTrajectoryTrackingErrors(self.odein)

    self.r_tran = self.computeReferenceCommandInputOuterLoop()

    self.x_ref_tran_dot = self.computeReferenceModelOuterLoop()

    self.mu_PD_baseline_tran = self.computeMuPDbaselineOuterLoop()

    self.Phi_adaptive_tran_augmented = self.computeRegressorVectorOuterLoop()

    self.mu_adaptive_tran = M_MRAC.computeControlLaw(
      self.K_hat_x_tran, self.x_tran,
      self.K_hat_r_tran, self.r_tran,
      self.Theta_hat_tran, self.Phi_adaptive_tran_augmented
    )

    self.mu_tran_raw = self.computeMuRawOuterLoop()

    # Update Adaptive Laws Outer Loop
    self.updateAdaptiveLawsOuterLoop()

    # Outer Loop Safety Mechanism
    self.mu_x, self.mu_y, self.mu_z = self.safety_mechanism.apply(self.mu_tran_raw)
    
    # Compute total thrust, desired roll angle, desired pitch angle
    (
    self.u1,
    self.roll_ref,
    self.pitch_ref
    ) = Control.computeU1RollPitchRef(
      self.mu_x, 
      self.mu_y, 
      self.mu_z, 
      self.gains.mass_total_estimated,
      self.fp.G_acc,
      self.odein.yaw_ref
    )

    # Computes roll/pitch reference dot and ddot using state-space differentiators.
    (
    self.internal_state_differentiator_phi_ref_diff,
    self.internal_state_differentiator_theta_ref_diff,
    self.angular_position_ref_dot,
    self.angular_position_ref_ddot
    ) = Control.computeAngularReferenceSignals(
      self.fp,
      self.odein,
      self.roll_ref,
      self.pitch_ref,
      self.state_phi_ref_diff,
      self.state_theta_ref_diff
    ) 

    # Computes angular error and its derivative
    (
    self.angular_error,
    self.angular_position_dot,
    self.angular_error_dot
    ) = Control.computeAngularErrorAndDerivative(
      self.odein,
      self.roll_ref,
      self.pitch_ref,
      self.angular_position_ref_dot
    )

    (self.omega_cmd,
     self.omega_cmd_dot
    ) = self.computeOmegaCmdAndOmegaCmdDotInnerLoop()

    self.omega_ref_dot = self.computeReferenceModelInnerLoop()

    self.r_rot = self.computeReferenceCommandInputInnerLoop()

    self.Moment_baseline_PI = self.computeMomentPIbaselineInnerLoop()

    (self.Phi_adaptive_rot,
     self.Phi_adaptive_rot_augmented
    ) = self.computeRegressorVectorInnerLoop()

    # Update Adaptive Laws Inner Loop
    self.updateAdaptiveLawsInnerLoop()

    self.Moment_baseline = self.computeMomentBaselineInnerLoop()

    self.Moment_adaptive = M_MRAC.computeControlLaw(
      self.K_hat_x_rot, self.odein.angular_velocity,
      self.K_hat_r_rot, self.r_rot,
      self.Theta_hat_rot, self.Phi_adaptive_rot_augmented
    )

    (self.u2,
     self.u3,
     self.u4,
     _
    ) = self.computeU2_U3_U4()

    # Compute individual motor thrusts
    self.motor_thrusts = Control.computeMotorThrusts(self.fp, self.u1, self.u2, self.u3, self.u4)
  
  def ode(self, t, y):
    """
    Function called by RK4. Assumes `computeControlAlgorithm` was called
    at the beginning of the integration step to update internal state.
    """
    self.dy[0:2] = self.internal_state_differentiator_phi_ref_diff
    self.dy[2:4] = self.internal_state_differentiator_theta_ref_diff
    self.dy[4:10] = self.x_ref_tran_dot
    self.dy[10:13] = self.translational_position_in_I_ref - self.odein.translational_position_in_I_user
    self.dy[13:31] = self.K_hat_x_tran_dot.reshape(18,1)
    self.dy[31:40] = self.K_hat_r_tran_dot.reshape(9,1)
    self.dy[40:58] = self.Theta_hat_tran_dot.reshape(18,1)
    self.dy[58:61] = self.omega_ref_dot
    self.dy[61:70] = self.K_hat_x_rot_dot.reshape(9,1)
    self.dy[70:79] = self.K_hat_r_rot_dot.reshape(9,1)
    self.dy[79:97] = self.Theta_hat_rot_dot.reshape(18,1)
    self.dy[97:100] = self.odein.angular_velocity - self.omega_ref
    self.dy[100:103] = self.angular_error
    self.dy[103:106] = self.omega_ref - self.omega_cmd

    return np.array(self.dy)
  
  def updateAdaptiveLawsOuterLoop(self):
    """
    Update the outer loop adaptive laws with deadzone modification, e-modification, and
    projection operator (all if enabled).
    """
    # Precompute e^T*P*B and its norm for outer loop
    (eTranspose_P_B_tran,
     eTranspose_P_B_norm_tran
    ) = M_MRAC.compute_eTransposePB(self.e_tran, self.gains.P_tran, self.gains.B_tran)

    # Deadzone modification Outer Loop
    self.dead_zone_value_tran = M_MRAC.deadZoneModulationFunction(
      self.e_tran, self.gains.dead_zone_delta_tran, self.gains.dead_zone_e0_tran,
      self.gains.use_dead_zone_modification
    )

    # Outer Loop Adaptive Laws
    (self.K_hat_x_tran_dot,
     self.K_hat_r_tran_dot,
     self.Theta_hat_tran_dot
    ) = M_MRAC.computeAllRobustAdaptiveLaws(
      self.gains.Gamma_x_tran, self.x_tran,
      self.gains.Gamma_r_tran, self.r_tran,
      self.gains.Gamma_Theta_tran, self.Phi_adaptive_tran_augmented,
      eTranspose_P_B_tran,
      self.dead_zone_value_tran,
      self.gains.sigma_x_tran, self.gains.sigma_r_tran, self.gains.sigma_Theta_tran,
      eTranspose_P_B_norm_tran,
      self.K_hat_x_tran, self.K_hat_r_tran, self.Theta_hat_tran,
      self.gains.use_dead_zone_modification, self.gains.use_e_modification
    )

    # Projection Operator Outer Loop
    if self.gains.use_projection_operator:
      (self.K_hat_x_tran_dot,
       self.proj_op_activated_K_hat_x_tran
      ) = ProjectionOperator.Ellipsoid.projectionMatrix(
        self.K_hat_x_tran,
        self.K_hat_x_tran_dot,
        self.gains.x_e_x_tran,
        self.gains.S_x_tran,
        self.gains.epsilon_x_tran
      )

      (self.K_hat_r_tran_dot,
       self.proj_op_activated_K_hat_r_tran
      ) = ProjectionOperator.Ellipsoid.projectionMatrix(
        self.K_hat_r_tran,
        self.K_hat_r_tran_dot,
        self.gains.x_e_r_tran,
        self.gains.S_r_tran,
        self.gains.epsilon_r_tran
      )

      (self.Theta_hat_tran_dot,
       self.proj_op_activated_Theta_hat_tran
      ) = ProjectionOperator.Ellipsoid.projectionMatrix(
        self.Theta_hat_tran,
        self.Theta_hat_tran_dot,
        self.gains.x_e_Theta_tran,
        self.gains.S_Theta_tran,
        self.gains.epsilon_Theta_tran
      )

  def updateAdaptiveLawsInnerLoop(self):
    """
    Update the inner loop adaptive laws with deadzone modification, e-modification, and
    projection operator (all if enabled).
    """
    # Precompute e^T*P*B and its norm for inner loop
    (eTranspose_P_B_rot,
     eTranspose_P_B_norm_rot
    ) = M_MRAC.compute_eTransposePB(self.e_rot, self.gains.P_rot, self.gains.B_rot)

    # Deadzone modification Inner Loop
    self.dead_zone_value_rot = M_MRAC.deadZoneModulationFunction(
      self.e_rot, self.gains.dead_zone_delta_rot, self.gains.dead_zone_e0_rot,
      self.gains.use_dead_zone_modification
    )

    # Inner Loop Adaptive Laws
    (self.K_hat_x_rot_dot,
     self.K_hat_r_rot_dot,
     self.Theta_hat_rot_dot
    ) = M_MRAC.computeAllRobustAdaptiveLaws(
      self.gains.Gamma_x_rot, self.odein.angular_velocity,
      self.gains.Gamma_r_rot, self.r_rot,
      self.gains.Gamma_Theta_rot, self.Phi_adaptive_rot_augmented,
      eTranspose_P_B_rot,
      self.dead_zone_value_rot,
      self.gains.sigma_x_rot, self.gains.sigma_r_rot, self.gains.sigma_Theta_rot,
      eTranspose_P_B_norm_rot,
      self.K_hat_x_rot, self.K_hat_r_rot, self.Theta_hat_rot,
      self.gains.use_dead_zone_modification, self.gains.use_e_modification
    )

    # Projection Operator Inner Loop
    if self.gains.use_projection_operator:
      (self.K_hat_x_rot_dot,
       self.proj_op_activated_K_hat_x_rot
      ) = ProjectionOperator.Ellipsoid.projectionMatrix(
        self.K_hat_x_rot,
        self.K_hat_x_rot_dot,
        self.gains.x_e_x_rot,
        self.gains.S_x_rot,
        self.gains.epsilon_x_rot
      )

      (self.K_hat_r_rot_dot,
       self.proj_op_activated_K_hat_r_rot
      ) = ProjectionOperator.Ellipsoid.projectionMatrix(
        self.K_hat_r_rot,
        self.K_hat_r_rot_dot,
        self.gains.x_e_r_rot,
        self.gains.S_r_rot,
        self.gains.epsilon_r_rot
      )

      (self.Theta_hat_rot_dot,
       self.proj_op_activated_Theta_hat_rot
      ) = ProjectionOperator.Ellipsoid.projectionMatrix(
        self.Theta_hat_rot,
        self.Theta_hat_rot_dot,
        self.gains.x_e_Theta_rot,
        self.gains.S_Theta_rot,
        self.gains.epsilon_Theta_rot
      )

  def computePostIntegrationAlgorithm(self):
    pass