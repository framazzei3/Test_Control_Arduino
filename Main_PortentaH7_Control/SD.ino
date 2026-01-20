// ========== FUNCTIONS ==========
void initSDCard() {
    log("[INFO]: Mounting SD card...");
    int err = fs.mount(&block_device);

    if (err) {
        log("[WARN]: No filesystem found, formatting...");
        err = fs.reformat(&block_device);
        if (err) {
            log("[ERROR]: Failed to format SD card");
            sdCardInitialized = false;
            return;  // Esce senza valore di ritorno
        }
        // Prova a montare di nuovo dopo la formattazione
        err = fs.mount(&block_device);
        if (err) {
            log("[ERROR]: Failed to mount after formatting");
            sdCardInitialized = false;
            return;
        }
    }

    // Crea la cartella con la data
    String folderPath = "/fs/" + getDateFolder();
    if (mkdir(folderPath.c_str(), 0777) != 0 && errno != EEXIST) {
        log("[ERROR]: Failed to create date folder");
        sdCardInitialized = false;
        return;
    }
    
    sdCardInitialized = true;  // Tutto ok!
}

void writeCSVHeader(FILE* file) {
    // fprintf(file, "Timestamp_ms,NTP_DateTime,ControlMode,");
    fprintf(file,
    "millis,time,mode,"
    // "posX,posY,posZ,roll,pitch,yaw,"
    "dt_control,v_raw,v_hat,v_ref,z_ref,z_meas,"
    "u_intVertical,u_pi,"
    "leftThrust,rightThrust,verticalThrust\n"
    );

}

void logSensorData() {

    // Esci subito se la SD non è disponibile
    if (!sdCardInitialized) return;

    static unsigned long lastLogTime = 0;
    const unsigned long logInterval = 100; // Log ogni 100ms


    unsigned long currentTime = millis();
    if (currentTime - lastLogTime < logInterval) return;
    lastLogTime = currentTime;

    // Initialize filename on first call
    if (filename == "") {
        String folderPath = "/fs/" + getDateFolder();
        filename = folderPath + "/" + getDateTimeFilename();
        
        // Create file and write header
        FILE* file = fopen(filename.c_str(), "w");
        if (!file) {
            log("[ERROR] Failed to create CSV file");
            return;
        }
        writeCSVHeader(file);
        fclose(file);
        log("[INFO] Created new log file: " + filename);
    }
    
    // Open file in append mode
    FILE* file = fopen(filename.c_str(), "a");
    if (!file) {
        log("[ERROR] Failed to open CSV file for writing");
        return;
    }

    // ================================
    //       SCRITTURA CSV
    // ================================

    // Timestamp and control mode
    fprintf(file, "%lu,%s,%s,", 
            currentTime, 
            getCurrentTimeString().c_str(), 
            getModeName(currentControlMode).c_str());

    // Battery data
    fprintf(file, "%.2f,%.1f,",
            batteryStatus.voltage, 
            batteryStatus.percent);

    // Position and orientation
    // fprintf(file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
    //         mocapData.posX, mocapData.posY, mocapData.posZ,
    //         mocapData.roll, mocapData.pitch, mocapData.yaw);


    fprintf(file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
            dt,
            v_raw,         // velocità non filtrata
            v_hat,         // velocità filtrata
            v_ref,         // target velocità
            z_ref,         // quota target
            z_meas, // quota misurata
            u_intVertical, // integratore
            u_pi           // u
    );

        // ToF (VL53L4CX)
    fprintf(file, "%.3f,", tofAltitude_m);   // altitudine misurata dal sensore ToF (metri)

    // motor trhusts in manual mode
    fprintf(file, "%.3f,%.3f,%.3f\n", leftThrust, rightThrust, verticalThrust);
   
    fflush(file);
    fclose(file);

}




