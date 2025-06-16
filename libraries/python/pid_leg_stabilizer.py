class PIDLegStabilizer:
    def __init__(
        self,
        kp_pitch=0.0,
        ki_pitch=0.0,
        kd_pitch=0.0,
        kp_roll=0.0,
        ki_roll=0.0,
        kd_roll=0.0,
    ):
        # Pitch PID gains
        self.kp_pitch = kp_pitch
        self.ki_pitch = ki_pitch
        self.kd_pitch = kd_pitch

        # Roll PID gains
        self.kp_roll = kp_roll
        self.ki_roll = ki_roll
        self.kd_roll = kd_roll

        # Internal state
        self.pitch_integral = 0.0
        self.roll_integral = 0.0
        self.prev_pitch_error = 0.0
        self.prev_roll_error = 0.0

        self.pitch_setpoint = 0.0
        self.roll_setpoint = 0.0

    def set_setpoint(self, pitch_setpoint, roll_setpoint):
        """Set desired pitch and roll angles (in degrees)."""
        self.pitch_setpoint = pitch_setpoint
        self.roll_setpoint = roll_setpoint

    def compute_leg_deltas(self, pitch, roll, dt):
        """Compute Δy offsets for each leg based on body pitch and roll."""
        # Errors
        pitch_error = self.pitch_setpoint - pitch
        roll_error = self.roll_setpoint - roll

        # Integrals
        self.pitch_integral += pitch_error * dt
        self.roll_integral += roll_error * dt

        # Derivatives
        pitch_derivative = (pitch_error - self.prev_pitch_error) / dt
        roll_derivative = (roll_error - self.prev_roll_error) / dt

        # PID outputs
        pitch_output = (
            self.kp_pitch * pitch_error
            + self.ki_pitch * self.pitch_integral
            + self.kd_pitch * pitch_derivative
        )

        roll_output = (
            self.kp_roll * roll_error
            + self.ki_roll * self.roll_integral
            + self.kd_roll * roll_derivative
        )

        # Save previous errors
        self.prev_pitch_error = pitch_error
        self.prev_roll_error = roll_error

        # Compute leg delta_y offsets (same pattern, outputs just vary)
        delta_y_rf = -pitch_output + roll_output  # Right Front
        delta_y_lf = -pitch_output - roll_output  # Left Front
        delta_y_rb = +pitch_output + roll_output  # Right Back
        delta_y_lb = +pitch_output - roll_output  # Left Back

        return [delta_y_lf, delta_y_rf, delta_y_lb, delta_y_rb]
