import math
import numpy as np  

class M_MRAC:
  @staticmethod
  def computeControlLaw(
    K_hat_x: np.ndarray,
    x: np.ndarray,
    K_hat_r: np.ndarray,
    r: np.ndarray,
    Theta_hat: np.ndarray,
    Phi: np.ndarray
    ) -> np.ndarray:
    """
    Compute the classical MRAC control law.
    """
    control_input = (
      K_hat_x.T * x
      + K_hat_r.T * r
      - Theta_hat.T * Phi
    )

    return control_input

  @staticmethod
  def computeAdaptiveLaw(
      Gamma_gain: np.ndarray,
      pi_vector: np.ndarray,
      eTranspose_P_B: np.ndarray
    ) -> np.ndarray:
    """
    Compute the classical MRAC adaptive law.
    """
    K_hat_state_dot = Gamma_gain * pi_vector * eTranspose_P_B
    return K_hat_state_dot

  @staticmethod
  def compute_eTransposePB(e: np.ndarray, P: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Compute eᵀ * P * B and its norm.

    Parameters:
      e (np.ndarray): Error vector (column vector).
      P (np.ndarray): Symmetric positive definite matrix.
      B (np.ndarray): Input matrix.

    Returns:
      tuple:
        - eTranspose_P_B (np.ndarray): The product eᵀ * P * B (1 x m vector if B has m columns).
        - eTranspose_P_B_norm (float): The Euclidean norm (2-norm) of eᵀ * P * B.
    """
    eTranspose_P_B = e.T * P * B
    eTranspose_P_B_norm = float(np.linalg.norm(eTranspose_P_B))
    return eTranspose_P_B, eTranspose_P_B_norm

  @staticmethod
  def computeAllAdaptiveLaws(
    Gamma_x,
    x,
    Gamma_r,
    r,
    Gamma_Theta,
    Phi_regressor_vector,
    eTranspose_P_B
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute all the classical MRAC adaptive laws

    Returns:
      - (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot)
    """

    K_hat_x_dot = M_MRAC.computeAdaptiveLaw(-Gamma_x, x, eTranspose_P_B)
    K_hat_r_dot = M_MRAC.computeAdaptiveLaw(-Gamma_r, r, eTranspose_P_B)
    Theta_hat_dot = M_MRAC.computeAdaptiveLaw(Gamma_Theta, Phi_regressor_vector, eTranspose_P_B)

    return (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot)
  
  @staticmethod
  def deadZoneModulationFunction(e_vector: np.ndarray, delta: float, e_0: float, use_deadzone: bool) -> float:
    """
    Smooth dead-zone modulation function for MRAC.

    Reference: E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.2.1

    Parameters:
      - e_vector (np.ndarray): Tracking error vector.
      - delta (float): Must satisfy 0 < delta < 1. Characterizes the slope of the modulation function.
      - e_0 (float): Must be > 0. Defines the dead-zone threshold. The dead-zone modification stops the
      adaptation process when the norm of the tracking error becomes smaller than the prescribed value e_0.
      - use_deadzone (bool): If False, the function returns 1.0 (i.e., no dead-zone effect).

    Returns:
      float: Modulation coefficient between 0.0 and 1.0.
    """
    if not use_deadzone:
      return 1.0

    norm_e = np.linalg.norm(e_vector)
    coeff = (norm_e - delta * e_0) / ((1.0 - delta) * e_0)
    result = float(max(0.0, min(1.0, coeff)))
    return result
  
  @staticmethod
  def computeRobustAdaptiveLaw(
      Gamma_gain: np.ndarray,
      dead_zone_value: float,
      pi_vector: np.ndarray,
      eTranspose_P_B: np.ndarray,
      sigma_gain: float,
      eTranspose_P_B_norm: float,
      K_hat_state: np.ndarray,
      use_deadzone: bool,
      use_emodification: bool
    ) -> np.ndarray:
    """
    Compute the MRAC adaptive law with OPTIONAL dead-zone modification and e-modification capabilities.
    """
    modulation_factor = dead_zone_value if use_deadzone else 1.0
    classic_term = pi_vector * eTranspose_P_B

    if use_emodification:
      emod_term = sigma_gain * eTranspose_P_B_norm * K_hat_state
      update_term = classic_term - emod_term
    else:
      update_term = classic_term

    K_hat_state_dot = Gamma_gain * modulation_factor * update_term
    return K_hat_state_dot
  
  @staticmethod
  def computeAllRobustAdaptiveLaws(
    Gamma_x: np.ndarray,
    x: np.ndarray,
    Gamma_r: np.ndarray,
    r: np.ndarray,
    Gamma_Theta: np.ndarray,
    Phi_regressor_vector: np.ndarray,
    eTranspose_P_B: np.ndarray,
    dead_zone_value: float,
    sigma_x: float,
    sigma_r: float,
    sigma_Theta: float,
    eTranspose_P_B_norm: float,
    K_hat_x: np.ndarray,
    K_hat_r: np.ndarray,
    Theta_hat: np.ndarray,
    use_deadzone: bool,
    use_emodification: bool
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute all MRAC adaptive laws with OPTIONAL dead-zone and e-modification.

    Returns:
      - (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot)
    """
    K_hat_x_dot = M_MRAC.computeRobustAdaptiveLaw(
      -Gamma_x,
      dead_zone_value,
      x,
      eTranspose_P_B,
      sigma_x,
      eTranspose_P_B_norm,
      K_hat_x,
      use_deadzone,
      use_emodification
    )

    K_hat_r_dot = M_MRAC.computeRobustAdaptiveLaw(
      -Gamma_r,
      dead_zone_value,
      r,
      eTranspose_P_B,
      sigma_r,
      eTranspose_P_B_norm,
      K_hat_r,
      use_deadzone,
      use_emodification
    )

    Theta_hat_dot = M_MRAC.computeRobustAdaptiveLaw(
      Gamma_Theta,
      dead_zone_value,
      Phi_regressor_vector,
      eTranspose_P_B,
      sigma_Theta,
      eTranspose_P_B_norm,
      Theta_hat,
      use_deadzone,
      use_emodification
    )

    return (K_hat_x_dot, K_hat_r_dot, Theta_hat_dot)