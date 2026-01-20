//==================================================
//                 VELOCITY ESTIMATION PARAMETERS 
//==================================================
float z_prev = 0.0f;
float v_raw  = 0.0f;
float v_hat  = 0.0f;
float z_meas = 0.0f;

float dt = 0.00f;
bool firstRun = true;

// Low-pass filter factor (0.8–0.95 typical)
const float alpha_v = 0.7f;

//==================================================
//                 CONTROL LIMITS                    
//==================================================
const float U_MIN = -1.0f;
const float U_MAX =  1.0f;
const float V_MAX =  1.0f;   // max vertical speed [m/s]

//==================================================
//              VELOCITY PI CONTROLLER               
//==================================================
// PID variables (velocity loop) - 
float v_ref    = 0.0f;

// PI gains (UP / DOWN)
float Kp_up   = 0.6f;
float Ki_up   = 0.09f;
float Kp_down = 3.1f;
float Ki_down = 1.5f;
float deltaz = 0.0f;
//==================================================
//               ALTITUDE CONTROL                   
//==================================================
float z_ref = 0.5f;   // target altitude [m]
float Kp_z  = 0.6f;   // altitude -> velocity gain


unsigned long lastTimeVertical = 0;
float u_intVertical = 0.0f;
float u_pi = 0.0f;   // comando totale PI 


//==================================================
//                  SETUP                         
//==================================================

void setupVerticalControl() {
  lastTimeVertical = millis();
  log("[INIT] Vertical control initialized");
}

//==================================================
//                MAIN CONTROL LOOP                 
//==================================================

void applyVerticalControl() {
  // Safety
  // if (!mocapData.mocapValid) {
  //   log("[WARN] MoCap invalid - switching to SAFETY");
  //   currentControlMode = MODE_SAFETY;
  //   return;
  // }

  // Timing (50 ms)

  static unsigned long lastExec = 0;
  unsigned long now = millis();

  if (now - lastExec < 100) return; // 50 ms = 20 Hz
  dt = (now - lastExec) * 1e-3f;
  lastExec = now;

  // --- Acquire altitude from mocap ---
  // float z_meas = mocapData.posZ;
  z_meas = tofAltitude_m;

  // ======================
  // VELOCITY ESTIMATION
  // ======================
  if (firstRun) {
    z_prev = z_meas;
    v_hat = 0.0f;
    firstRun = false;
  } else {
    v_raw = (z_meas - z_prev) / dt;
    z_prev = z_meas;

    v_hat = alpha_v * v_hat + (1.0f - alpha_v) * v_raw;
  }

  // ======================
  // ALTITUDE -> VELOCITY (OUTER LOOP)
  // ======================
  v_ref = Kp_z * (z_ref - z_meas);
  v_ref = constrain(v_ref, -V_MAX, V_MAX);

  // ======================
  // GAIN SCHEDULING
  // ======================
  float Kp_v = (v_ref >= 0.0f) ? Kp_up : Kp_down;
  float Ki_v = (v_ref >= 0.0f) ? Ki_up : Ki_down;

  // ======================
  // VELOCITY CONTROL (INNER LOOP)
  // ======================

  float e_v = v_ref - v_hat;         // errore velocità
  u_intVertical  += Ki_v * dt * e_v; // integratore

  u_pi = Kp_v * e_v + u_intVertical ; // comando totale

  // saturazione + antiwindup
  if (u_pi > U_MAX) {
    u_pi = U_MAX;
    u_intVertical  -= Ki_v * dt * e_v;
  } else if (u_pi < U_MIN) {
    u_pi = U_MIN;
    u_intVertical  -= Ki_v * dt * e_v;
  }

  // comando motore verticale
  setVerticalMotorSpeed(u_pi);
}


void logVerticalControl() {
  String msg = "[VERT] ";
  msg += "z_meas=" + String(z_meas, 3) + ", ";
  msg += "v_raw="  + String(v_raw, 3)  + ", ";
  msg += "v_hat="  + String(v_hat, 3)  + ", ";
  msg += "v_ref="  + String(v_ref, 3)  + ", ";
  msg += "Kp="     + String((v_ref >= 0 ? Kp_up : Kp_down), 3) + ", ";
  msg += "Ki="     + String((v_ref >= 0 ? Ki_up : Ki_down), 3) + ", ";
  msg += "e_v="    + String(v_ref - v_hat, 3) + ", ";
  msg += "u_int="  + String(u_intVertical, 3) + ", ";
  msg += "u_pi="   + String(u_pi, 3);

  log(msg);
}

