import math
import numpy as np  
from acsl_pychrono.control.outerloop_safetymech import OuterLoopSafetyMechanism
from acsl_pychrono.control.BackStepping.backstepping_gains import BackSteppingGains
from acsl_pychrono.simulation.ode_input import OdeInput
from acsl_pychrono.simulation.flight_params import FlightParams
from acsl_pychrono.control.control import Control

class BackStepping(Control):
  def __init__(self, gains: BackSteppingGains, ode_input: OdeInput, flight_params: FlightParams, timestep: float):
    super().__init__(odein=ode_input)
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
    self.state_phi_ref_diff = self.y[0:2]
    self.state_theta_ref_diff = self.y[2:4]
    self.integral_position_tracking = self.y[4:7]
    self.integral_angular_error = self.y[7:10]
 
    # Compute translational position error
    self.translational_position_error = Control.computeTranslationalPositionError(
      self.odein.translational_position_in_I,
      self.odein.translational_position_in_I_user
    )   
    
    # Compute Outer Loop
    self.computeOuterLoop()

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
    
    # Compute Inner Loop
    self.computeInnerLoop()

    # Compute individual motor thrusts
    self.motor_thrusts = Control.computeMotorThrusts(self.fp, self.u1, self.u2, self.u3, self.u4)
  
  def ode(self, t, y):
    """
    Function called by RK4. Assumes `computeControlAlgorithm` was called
    at the beginning of the integration step to update internal state.
    """
    self.dy[0:2] = self.internal_state_differentiator_phi_ref_diff
    self.dy[2:4] = self.internal_state_differentiator_theta_ref_diff
    self.dy[4:7] = self.translational_position_error
    self.dy[7:10] = self.angular_error

    return np.array(self.dy)
  
  def computeOuterLoop(self):
    

    e1 = self.translational_position_error  # (3x1)
        # Use K1_tran, K2_tran if provided; fallback to KP_tran, KD_tran
    K1 = getattr(self.gains, "K1_tran", self.gains.K1)
    K2 = getattr(self.gains, "K2_tran", self.gains.K2)

    v    = self.odein.translational_velocity_in_I            # (3x1)
    v_d  = self.odein.translational_velocity_in_I_user - K1 @ e1
    e2   = v - v_d
    e1_dot = e2 - K1 @ e1

    # --- Drag inversion (your original code, kept) ---
    R_L2G, R_G2L = Control.computeRotationMatrices(self.odein.roll, self.odein.pitch, self.odein.yaw)
    v_body = R_G2L * v
    v_body_norm = float(np.linalg.norm(v_body))
    drag_force_body = (
        -0.5 * self.gains.air_density_estimated * self.gains.surface_area_estimated
        * self.gains.drag_coefficient_matrix_estimated * v_body * v_body_norm
    )
    drag_force_inertial = R_L2G * drag_force_body
    dynamic_inversion = -drag_force_inertial

        # Backstepping acceleration command (in inertial/NED)
    a_cmd = (
        self.odein.translational_acceleration_in_I_user
        - K1 @ e1_dot
        - e1
        - K2 @ e2
    )

        # Force request (+ drag inversion). DO NOT add gravity here.
    self.mu_tran_raw = (
        self.gains.mass_total_estimated * a_cmd + dynamic_inversion
    ).reshape(3, 1)



  def computeInnerLoop(self):
    # """
    # Computes control moments (u2, u3, u4) for the inner loop.
    # """

    # --- Inertia & body rates ---
    I = np.asarray(self.gains.I_matrix_estimated, dtype=float)              # (3x3)
    omega = np.asarray(self.odein.angular_velocity, dtype=float).reshape(3,1)  # [p,q,r]^T

    # Gyro cancellation τ_gyro = ω × (I ω)
    gyro_term = np.cross(omega.ravel(), (I @ omega).ravel()).reshape(3,1)

    # --- Errors & reference derivatives (3x1 each) ---
    e1          = np.asarray(self.angular_error, dtype=float).reshape(3,1)                 # e1 = η - η_d
    eta_dot_ref = np.asarray(self.angular_position_ref_dot, dtype=float).reshape(3,1)      # η̇_d
    eta_ddot_ref= np.asarray(self.angular_position_ref_ddot, dtype=float).reshape(3,1)     # η̈_d

    # --- Backstepping gains C1, C2 (diagonal, >0). Use provided or safe defaults. ---
    def _get_diag3(attr_name, fallback):
        val = getattr(self.gains, attr_name, None)
        if val is None:
            return np.array(fallback, float)
        arr = np.asarray(val, float).reshape(-1)
        if arr.size == 1: return np.array([arr[0], arr[0], arr[0]])
        if arr.size == 3: return arr
        if arr.shape == (3,3): return np.diag(arr)
        return np.array(fallback, float)

    C1_diag = _get_diag3("C1_rot", [12.0, 12.0, 6.0])
    C2_diag = _get_diag3("C2_rot", [48.0, 48.0, 30.0])
    C1 = np.diag(C1_diag); C2 = np.diag(C2_diag)
    I3 = np.eye(3)

    # --- Soft fence on angle error (prevents violent commands when far away) ---
    E_ANG_MAX = np.deg2rad(35.0)
    e1_sat = np.clip(e1, -E_ANG_MAX, E_ANG_MAX)

    # --- Exact kinematics: J, J^{-1}, J̇ ---
    phi   = float(self.odein.roll)
    theta = float(self.odein.pitch)

    use_fallback = False
    try:
        J = np.asarray(self.computeJacobian(phi, theta), dtype=float)            # (3x3)
        # η̇(meas) = J^{-1} ω  (solve is more stable than explicit inverse)
        eta_dot_meas = np.linalg.solve(J, omega).reshape(3,1)                    # (3x1)
        roll_dot   = float(eta_dot_meas[0,0])
        pitch_dot  = float(eta_dot_meas[1,0])
        Jdot = np.asarray(self.computeJacobianDot(phi, theta, roll_dot, pitch_dot), dtype=float)
        #print("Not nominal case")
        # Guard against numerical nasties near cos(theta) ≈ 0
        if not np.isfinite(J).all() or not np.isfinite(Jdot).all():
            use_fallback = True
            #print("Nominal case is using")

    except Exception:
        use_fallback = True
        #print("Nominal case is using")

    # --- Backstepping law ---
    if not use_fallback:
        # Virtual rate and second error in Euler-rate space
        v  = eta_dot_ref - (C1 @ e1_sat)         # v = η̇_d - C1 e1
        e2 = eta_dot_meas - v                    # e2 = η̇ - v

        # PURE backstepping desired Euler acceleration:
        #   η̈_cmd = η̈_d - (C1+C2) e2 + (C1^2 - I) e1
        eta_ddot_cmd = eta_ddot_ref - ((C1 + C2) @ e2) + ((C1 @ C1 - I3) @ e1_sat)

        # Map to body-rate acceleration using exact kinematics:
        #   ω̇_cmd = J η̈_cmd + J̇ η̇   (use measured η̇ for consistency)
        omega_dot_cmd = (J @ eta_ddot_cmd) + (Jdot @ eta_dot_meas)

    else:
        # ---- Fallback: small-angle pure backstepping (ω ≈ η̇) ----
        v  = eta_dot_ref - (C1 @ e1_sat)
        e2 = omega - v
        eta_ddot_cmd = eta_ddot_ref - ((C1 + C2) @ e2) + ((C1 @ C1 - I3) @ e1_sat)
        # small-angle: ω̇_cmd ≈ η̈_cmd
        omega_dot_cmd = eta_ddot_cmd

    # --- Safety clamps (keep demands physical; tune as needed) ---
    MAX_ACCEL = np.deg2rad(6000.0)                 # rad/s^2 per axis
    omega_dot_cmd = np.clip(omega_dot_cmd, -MAX_ACCEL, MAX_ACCEL)

    # Consistent torque clamp with MAX_ACCEL
    tau_ff  = (I @ omega_dot_cmd).reshape(3,1)
    tau_cap = (I @ np.full((3,1), MAX_ACCEL)).reshape(3,1)
    tau     = gyro_term + np.clip(tau_ff, -np.abs(tau_cap), np.abs(tau_cap))

    # --- Output torques ---
    self.Moment = tau
    self.u2 = float(self.Moment[0, 0])   # τ_φ
    self.u3 = float(self.Moment[1, 0])   # τ_θ
    self.u4 = float(self.Moment[2, 0])   # τ_ψ



    
    #--- common precompute ---
    

  def computePostIntegrationAlgorithm(self):
    pass