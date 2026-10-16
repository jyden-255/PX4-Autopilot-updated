# MATEK F405-VTOL board configuration
# Based on PX4 STM32F405 target with VTOL extensions
# Chip: STM32F405RGT6  (1MB Flash, 192KB SRAM)

px4_add_board(
    PLATFORM nuttx
    VENDOR matek
    MODEL f405-vtol
    LABEL "MATEK F405 VTOL"
    UAVCAN_INTERFACES 0

    SERIAL_PORTS
        GPS1:/dev/ttyS3      # UART4  – GPS/external wind sensor
        TEL1:/dev/ttyS1      # USART2 – Telemetry (GCS link)
        TEL2:/dev/ttyS2      # USART3 – MAVLink companion
        RC_SERIAL:/dev/ttyS0 # USART1 – RC input (SBUS/CRSF)

    DRIVERS
        adc/board_adc
        barometer/ms5611          # Barometric altitude
        differential_pressure/ms4525  # Airspeed sensor
        gps
        imu/invensense/icm42688p  # Primary IMU (6-axis)
        imu/invensense/mpu6000    # Secondary IMU
        magnetometer/ist8310
        magnetometer/qmc5883l
        power_monitor/ina226      # Battery current/voltage
        pwm_out
        rc_input
        telemetry/hott_telemetry

    MODULES
        # Core flight stack
        commander
        navigator
        sensors
        ekf2
        land_detector
        dataman
        rc_update
        manual_control
        events

        # VTOL-specific
        vtol_att_control          # Tiltrotor/tailsitter transitions

        # Multicopter controllers (hover mode)
        mc_att_control
        mc_rate_control
        mc_pos_control
        mc_hover_thrust_estimator

        # Fixed-wing controllers (cruise mode)
        fw_att_control
        fw_rate_control
        fw_lateral_longitudinal_control
        airspeed_selector

        # Flight modes
        flight_mode_manager

        # Mission / Navigation
        mission

        # ── NEW: Custom enhancement modules ────────────────────────
        crosswind_compensator     # Real-time wind fusion + crab-angle correction
        autonomous_navigator      # Dijkstra path planner + offline cache + approval gate

        # Logging & comms
        logger
        mavlink
        uxrce_dds_client

        # Utilities
        load_mon
        temperature_compensation
        battery_status

    SYSTEMCMDS
        bl_update
        dumpfile
        hardfault_log
        led_control
        mixer
        motor_test
        mtd
        nshterm
        param
        perf
        reboot
        sd_bench
        topic_listener
        tune_control
        uorb
        ver
        work_queue

    ROMFSFILES
        etc/init.d/rc.board_mavlink
)
