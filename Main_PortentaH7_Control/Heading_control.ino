// ====== PID Settings for Heading Control (Yaw) ======

// ========== PARAMETRI CONTROLLO ANGOLARE (outer loop) ==========
const float Kp_psi = 1.2;     // guadagno angolo → yaw rate
const float W_MAX  = 1.5;     // saturazione yaw-rate reference (rad/s)

// ========== PARAMETRI CONTROLLO VELOCITÀ ANGOLARE (inner PI loop) ==========
const float Kp_w = 0.8;
const float Ki_w = 0.3;

// ========== FILTRO VELOCITÀ ANGOLARE ==========
const float alpha_w = 0.85;   // filtro esponenziale low-pass

// ========== VARIABILI ==========
float yawSetpoint = 0.0;
float yawInput    = 0.0;

float w_hat       = 0.0;      // yaw rate filtrato
float w_prev      = 0.0;

// Heading control
unsigned long lastTimeYaw = 0;
float u_intYaw = 0.0f;

void setupHeadingPID() {
  lastTimeYaw = millis();        // importantissimo per dt
  yawSetpoint = mocapData.yaw;
}


void applyHeadingPID() {

  // Safety check
  if (!mocapData.mocapValid) {
    log("[WARN] MoCap data invalid - switching to SAFETY mode");
    currentControlMode = MODE_SAFETY;
    return;
  }

  // ---- 1. Timestamp ----
  unsigned long now = millis();
  float dt = (now - lastTimeYaw) * 1e-3f;
  lastTimeYaw = now;

  if (dt <= 0.0f || dt > 0.2f)  // failsafe: dt troppo grande
    return;

  // ---- 2. Lettura yaw ----
  yawInput = mocapData.yaw;

  // ---- 3. Stima yaw rate w_hat ----
  float w_raw = (yawInput - w_prev) / dt;
  w_prev = yawInput;

  // filtro esponenziale
  w_hat = alpha_w * w_hat + (1.0f - alpha_w) * w_raw;

  // ---- 4. Outer loop: angle → yaw rate reference ----
  float e_psi = yawSetpoint - yawInput;

  // wrapping in [-180, 180]
  if (e_psi > 180)  e_psi -= 360;
  if (e_psi < -180) e_psi += 360;

  float w_ref = Kp_psi * e_psi;

  // saturazione
  if (w_ref > W_MAX)  w_ref = W_MAX;
  else if (w_ref < -W_MAX) w_ref = -W_MAX;

  // ---- 5. Inner loop PI su yaw rate ----
  float e_w = w_ref - w_hat;

  // integratore
  u_intYaw += Ki_w * dt * e_w;

  // controllo totale
  float u_pi = Kp_w * e_w + u_intYaw;

  // ---- 6. Saturazione e anti-windup ----
  if (u_pi > 1.0f) {
    u_pi = 1.0f;
    u_intYaw -= Ki_w * dt * e_w;   // antiwindup
  }
  else if (u_pi < -1.0f) {
    u_pi = -1.0f;
    u_intYaw -= Ki_w * dt * e_w;   // antiwindup
  }

  // ---- 7. Output ai motori ----
  setLeftMotorSpeed(-u_pi);
  setRightMotorSpeed( u_pi);
}