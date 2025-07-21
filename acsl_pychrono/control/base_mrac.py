import math
import numpy as np
from numpy import linalg as LA

from acsl_pychrono.simulation.ode_input import OdeInput
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.control.control import Control
from acsl_pychrono.control.base_mrac_gains import BaseMRACGains

class BaseMRAC():  
  def __init__(self, odein: OdeInput, gains: BaseMRACGains) -> None:
    self.odein: OdeInput = odein
    self.gains: BaseMRACGains = gains
    self.e_tran = np.zeros((6, 1))
    self.e_rot = np.zeros((3, 1))
    self.x_ref_tran = np.zeros((6, 1))
    self.x_ref_tran_dot = np.zeros((6, 1))
    self.integral_position_tracking_ref = np.zeros((3, 1))
    self.mu_PD_baseline_tran = np.zeros((3, 1))
    self.mu_adaptive_tran = np.zeros((3, 1))
    self.omega_ref = np.zeros((3, 1))
    self.omega_ref_dot = np.zeros((3, 1))
    self.r_tran = np.zeros((3, 1))
    self.angular_position_dot = np.zeros((3, 1))
    self.angular_error = np.zeros((3, 1))
    self.integral_angular_error = np.zeros((3, 1))
    self.angular_position_ref_dot = np.zeros((3, 1))
    self.angular_error_dot = np.zeros((3, 1))
    self.angular_position_ref_ddot = np.zeros((3, 1))
    self.omega_cmd = np.zeros((3, 1))
    self.integral_e_omega_ref_cmd = np.zeros((3, 1))
    self.omega_cmd_dot = np.zeros((3, 1))
    self.integral_e_rot = np.zeros((3, 1))
    self.Moment_baseline_PI = np.zeros((3, 1))
    self.Moment_baseline = np.zeros((3, 1))
    self.Moment_adaptive = np.zeros((3, 1))

  def reshapeAdaptiveGainsToMatricesMRAC(self):
    """
    Reshapes all gain parameters to their correct (row, col) shape and converts them to np.matrix.
    This is intended to be called once after loading or updating gains stored as flat arrays.
    """
    self.K_hat_x_tran = np.matrix(self.K_hat_x_tran.reshape(6,3))
    self.K_hat_r_tran = np.matrix(self.K_hat_r_tran.reshape(3,3))
    self.Theta_hat_tran = np.matrix(self.Theta_hat_tran.reshape(6,3))
    self.K_hat_x_rot = np.matrix(self.K_hat_x_rot.reshape(3,3))
    self.K_hat_r_rot = np.matrix(self.K_hat_r_rot.reshape(3,3))
    self.Theta_hat_rot = np.matrix(self.Theta_hat_rot.reshape(6,3))

  def reshapeAdaptiveGainsToMatricesTwoLayerMRAC(self):
    """
    Reshapes all gain parameters to their correct (row, col) shape and converts them to np.matrix.
    This is intended to be called once after loading or updating gains stored as flat arrays.
    """
    self.reshapeAdaptiveGainsToMatricesMRAC()
    self.K_hat_g_tran = np.matrix(self.K_hat_g_tran.reshape(6,3))
    self.K_hat_g_rot = np.matrix(self.K_hat_g_rot.reshape(3,3))

  def computeTrajectoryTrackingErrors(self, odein: OdeInput):
    """
    Computes translational and rotational tracking errors and extracts the reference position.
    Assumes self.odein and self.x_ref_tran / self.omega_ref are already set.
    """
    # State vector for translation: position + velocity
    self.x_tran = np.append(
      odein.translational_position_in_I,
      odein.translational_velocity_in_I,
      axis=0
    )
    
    # Compute trajectory tracking errors
    self.e_tran = self.x_tran - self.x_ref_tran
    self.e_rot = odein.angular_velocity - self.omega_ref
    
    # Extract reference position
    self.translational_position_in_I_ref = self.x_ref_tran[0:3]

  def computeReferenceCommandInputOuterLoop(self):
    r_tran = self.gains.mass_total_estimated * (
      - self.gains.KI_tran * self.integral_position_tracking_ref
      + self.odein.translational_acceleration_in_I_user
      + self.gains.KP_tran * self.odein.translational_position_in_I_user
      + self.gains.KD_tran * self.odein.translational_velocity_in_I_user
    )

    return r_tran
  
  def computeReferenceModelOuterLoop(self):
    x_ref_tran_dot = self.gains.A_ref_tran * self.x_ref_tran + self.gains.B_ref_tran * self.r_tran

    return x_ref_tran_dot
  
  def computeMuPDbaselineOuterLoop(self):
    mu_PD_baseline_tran = -self.gains.mass_total_estimated * (
      self.gains.KP_tran_PD_baseline * (self.odein.translational_position_in_I - self.translational_position_in_I_ref)
      + self.gains.KD_tran_PD_baseline * (self.odein.translational_velocity_in_I - self.x_ref_tran[3:6])
      - self.x_ref_tran_dot[3:6]
    )

    return mu_PD_baseline_tran
  
  def computeRegressorVectorOuterLoop(self):
    # Compute rotation matrices
    (R_from_loc_to_glob,
     R_from_glob_to_loc
    ) = Control.computeRotationMatrices(self.odein.roll, self.odein.pitch, self.odein.yaw)
    
    translational_velocity_in_J = R_from_glob_to_loc * self.odein.translational_velocity_in_I
    translational_velocity_in_J_norm = LA.norm(R_from_glob_to_loc * self.odein.translational_velocity_in_I)
    self.Phi_adaptive_tran = -0.5 * translational_velocity_in_J * translational_velocity_in_J_norm 

    Phi_adaptive_tran_augmented = np.matrix(np.block([[self.mu_PD_baseline_tran],
                                                      [self.Phi_adaptive_tran]]))
    
    return Phi_adaptive_tran_augmented
  
  def computeMuRawOuterLoop(self):
    mu_tran_raw = (
      self.mu_PD_baseline_tran
      + self.mu_adaptive_tran
    )

    return mu_tran_raw
  
  def computeOmegaCmdAndOmegaCmdDotInnerLoop(self):
    Jacobian_matrix = Control.computeJacobian(self.odein.roll, self.odein.pitch)

    Jacobian_matrix_dot = Control.computeJacobianDot(
      self.odein.roll,
      self.odein.pitch,
      self.angular_position_dot[0],
      self.angular_position_dot[1]
    )

    omega_cmd = Jacobian_matrix * (
      - self.gains.KP_rot * self.angular_error 
      - self.gains.KI_rot * self.integral_angular_error 
      + self.angular_position_ref_dot
    )
    
    omega_cmd_dot = (
      Jacobian_matrix_dot * (
        - self.gains.KP_rot * self.angular_error 
        - self.gains.KI_rot * self.integral_angular_error 
        + self.angular_position_ref_dot)
      + Jacobian_matrix * (
        - self.gains.KP_rot * self.angular_error_dot 
        - self.gains.KI_rot * self.angular_error 
        + self.angular_position_ref_ddot)
    )

    return omega_cmd, omega_cmd_dot
  
  def computeReferenceModelInnerLoop(self):
    omega_ref_dot = (
      - self.gains.K_P_omega_ref * (self.omega_ref - self.omega_cmd) 
      - self.gains.K_I_omega_ref * self.integral_e_omega_ref_cmd
      + self.omega_cmd_dot
    )

    return omega_ref_dot
  
  def computeReferenceCommandInputInnerLoop(self):
    r_rot = (
      self.gains.K_P_omega_ref * self.omega_cmd
      - self.gains.K_I_omega_ref * self.integral_e_omega_ref_cmd
      + self.omega_cmd_dot
    )

    return r_rot
  
  def computeMomentPIbaselineInnerLoop(self):
    Moment_baseline_PI = -self.gains.I_matrix_estimated * (
      self.gains.KP_rot_PI_baseline * self.e_rot
      + self.gains.KI_rot_PI_baseline * self.integral_e_rot 
      - self.omega_ref_dot
    )

    return Moment_baseline_PI
  
  def computeRegressorVectorInnerLoop(self):
    Phi_adaptive_rot = np.array([[self.odein.angular_velocity[1].item() * self.odein.angular_velocity[2].item()],
                                 [self.odein.angular_velocity[0].item() * self.odein.angular_velocity[2].item()],
                                 [self.odein.angular_velocity[0].item() * self.odein.angular_velocity[1].item()]])
    
    Phi_adaptive_rot_augmented = np.matrix(np.block([[self.Moment_baseline_PI],
                                                     [Phi_adaptive_rot]]))
    
    return Phi_adaptive_rot, Phi_adaptive_rot_augmented
  
  def computeMomentBaselineInnerLoop(self):
    Moment_baseline = np.cross(
      self.odein.angular_velocity.ravel(),
      (self.gains.I_matrix_estimated * self.odein.angular_velocity).ravel()
    ).reshape(3,1)

    return Moment_baseline
  
  def computeU2_U3_U4(self):
    Moment = self.Moment_baseline_PI + self.Moment_baseline + self.Moment_adaptive
    
    u2 = Moment[0].item()
    u3 = Moment[1].item()
    u4 = Moment[2].item()

    return u2, u3, u4, Moment
