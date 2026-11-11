$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $projectIncludes = @(
        '-Itests/host', '-Itests/host/can', '-Iconfig', '-Iapplication/chassis',
        '-Imodules/chassis_motion', '-Imodules/algorithm', '-Imodules/imu',
        '-Imodules/motor/dji', '-Imodules/motor', '-Imodules/daemon',
        '-Imodules/jam_detector', '-Imodules/diagnostics'
    )
    $projectSources = @(
        'tests/host/matrix.c', 'config/config.c', 'application/chassis/chassis_setup.c',
        'modules/chassis_motion/chassis_motion.c', 'modules/chassis_motion/kinematics.c',
        'modules/algorithm/kalman_filter.c'
    )
    New-Item -ItemType Directory -Path build/host_motion -Force | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST -Itests/host -Imodules/chassis_motion -Imodules/algorithm tests/chassis_control_permission_test.c tests/host/matrix.c modules/chassis_motion/chassis_motion.c modules/chassis_motion/kinematics.c modules/algorithm/kalman_filter.c -lm -o build/host_motion/control_permission.exe
    if ($LASTEXITCODE -ne 0) { throw 'Control permission compilation failed' }
    & .\build\host_motion\control_permission.exe
    if ($LASTEXITCODE -ne 0) { throw 'Control permission test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -Itests/host/dwt -Ibsp/dwt tests/dwt_recovery_test.c bsp/dwt/bsp_dwt.c -lm -o build/host_motion/dwt_recovery.exe
    if ($LASTEXITCODE -ne 0) { throw 'DWT recovery compilation failed' }
    & .\build\host_motion\dwt_recovery.exe
    if ($LASTEXITCODE -ne 0) { throw 'DWT recovery test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -Imodules/imu tests/imu_recovery_test.c modules/imu/imu_compensation.c -lm -o build/host_motion/imu_recovery.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU recovery compilation failed' }
    & .\build\host_motion\imu_recovery.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU recovery test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST -Itests/host -Imodules/chassis_motion -Imodules/algorithm tests/chassis_observation_guard_test.c tests/host/matrix.c modules/chassis_motion/chassis_motion.c modules/chassis_motion/kinematics.c modules/algorithm/kalman_filter.c -lm -o build/host_motion/observation_guard.exe
    if ($LASTEXITCODE -ne 0) { throw 'Observation guard compilation failed' }
    & .\build\host_motion\observation_guard.exe
    if ($LASTEXITCODE -ne 0) { throw 'Observation guard test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST -Itests/host -Imodules/chassis_motion -Imodules/algorithm tests/chassis_transient_guard_test.c tests/host/matrix.c modules/chassis_motion/chassis_motion.c modules/chassis_motion/kinematics.c modules/algorithm/kalman_filter.c -lm -o build/host_motion/transient_guard.exe
    if ($LASTEXITCODE -ne 0) { throw 'Transient guard compilation failed' }
    & .\build\host_motion\transient_guard.exe
    if ($LASTEXITCODE -ne 0) { throw 'Transient guard test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -fno-strict-aliasing -Itests/host -Imodules/algorithm tests/quaternion_dynamic_recovery_test.c tests/host/matrix.c modules/algorithm/quaternion_ekf.c modules/algorithm/kalman_filter.c -lm -o build/host_motion/quaternion_dynamic.exe
    if ($LASTEXITCODE -ne 0) { throw 'Dynamic attitude recovery compilation failed' }
    foreach ($dt in '0.001', '0.005', '0.010') {
        & .\build\host_motion\quaternion_dynamic.exe $dt
        if ($LASTEXITCODE -ne 0) { throw 'Dynamic attitude recovery test failed' }
    }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST -Itests/host -Imodules/chassis_motion -Imodules/algorithm tests/chassis_portability_test.c tests/host/matrix.c modules/chassis_motion/chassis_motion.c modules/chassis_motion/motion_port.c modules/chassis_motion/kinematics.c modules/algorithm/kalman_filter.c -lm -o build/host_motion/chassis_portability.exe
    if ($LASTEXITCODE -ne 0) { throw 'Portability test compilation failed' }
    & .\build\host_motion\chassis_portability.exe
    if ($LASTEXITCODE -ne 0) { throw 'Portability test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST -Itests/host/can -Imodules/can_comm -Imodules/daemon -Imodules/algorithm tests/can_comm_snapshot_test.c modules/can_comm/can_comm.c modules/algorithm/crc8.c modules/daemon/daemon.c -o build/host_motion/can_snapshot.exe
    if ($LASTEXITCODE -ne 0) { throw 'CAN snapshot compilation failed' }
    & .\build\host_motion\can_snapshot.exe
    if ($LASTEXITCODE -ne 0) { throw 'CAN snapshot test failed' }
    foreach ($board in 'ONE_BOARD', 'CHASSIS_BOARD', 'GIMBAL_BOARD') {
      foreach ($drive in 'OMNI_X', 'MECANUM', 'SWERVE') {
        & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST "-D$board" "-DCHASSIS_DRIVE_$drive" @projectIncludes tests/chassis_project_config_test.c @projectSources -lm -o build/host_motion/project_config.exe
        if ($LASTEXITCODE -ne 0) { throw 'Project configuration compilation failed' }
        & .\build\host_motion\project_config.exe
        if ($LASTEXITCODE -ne 0) { throw 'Project configuration test failed' }
      }
    }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -Itests/host/pid @projectIncludes -Iapplication/gimbal tests/gimbal_yaw_disturbance_test.c modules/algorithm/pid.c config/config.c -lm -o build/host_motion/gimbal_yaw_disturbance.exe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw disturbance test compilation failed' }
    & .\build\host_motion\gimbal_yaw_disturbance.exe
    if ($LASTEXITCODE -ne 0) { throw 'Yaw disturbance test failed' }
    foreach ($priority in 0, 1) {
      foreach ($calibrated in 0, 1) {
        $output = "build/host_motion/motion_${priority}_$calibrated.exe"
        # 通用 KF/调度用例的合成 IMU 位于原点；非零杆臂由下面的物理运动用例覆盖。
        & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST "-DMC_IMU_ACCEL_VALIDATED=$calibrated" "-DMC_WHEEL_LIMIT_ROTATION_PRIORITY=$priority" '-DMC_IMU_OFFSET_X_M=0.0f' -DMC_IMU_FOLLOWS_PITCH=0 @projectIncludes tests/chassis_motion_test.c @projectSources -lm -o $output
        if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
        & ".\$output"
        if ($LASTEXITCODE -ne 0) { throw 'Host motion test failed' }
      }
    }
    foreach ($onGimbal in 0, 1) {
      foreach ($calibrated in 0, 1) {
        $output = "build/host_motion/offset_${onGimbal}_${calibrated}.exe"
        & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST "-DMC_IMU_ACCEL_VALIDATED=$calibrated" "-DMC_IMU_ON_GIMBAL=$onGimbal" -DMC_IMU_FOLLOWS_PITCH=0 '-DMC_IMU_OFFSET_X_M=0.1f' '-DMC_IMU_OFFSET_Y_M=0.02f' '-DMC_YAW_CENTER_X_M=0.04f' '-DMC_YAW_CENTER_Y_M=-0.03f' @projectIncludes tests/chassis_motion_offset_test.c @projectSources -lm -o $output
        if ($LASTEXITCODE -ne 0) { throw 'Offset test compilation failed' }
        & ".\$output"
        if ($LASTEXITCODE -ne 0) { throw 'Motion offset test failed' }
      }
    }
    foreach ($calibrated in 0, 1) {
        $output = "build/host_motion/rigid3d_$calibrated.exe"
        & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST "-DMC_IMU_ACCEL_VALIDATED=$calibrated" @projectIncludes tests/chassis_motion_rigid3d_test.c @projectSources modules/imu/imu_compensation.c -lm -o $output
        if ($LASTEXITCODE -ne 0) { throw '3D lever test compilation failed' }
        & ".\$output"
        if ($LASTEXITCODE -ne 0) { throw '3D lever test failed' }
    }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -Imodules/imu tests/imu_compensation_test.c modules/imu/imu_compensation.c -lm -o build/host_motion/imu_compensation.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU compensation test compilation failed' }
    & .\build\host_motion\imu_compensation.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU compensation test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -Imodules/imu tests/imu_sampled_derivative_test.c modules/imu/imu_compensation.c -lm -o build/host_motion/imu_sampled_derivative.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sampled derivative compilation failed' }
    & .\build\host_motion\imu_sampled_derivative.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sampled derivative test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST @projectIncludes tests/chassis_fusion_test.c @projectSources -lm -o build/host_motion/chassis_fusion.exe
    if ($LASTEXITCODE -ne 0) { throw 'Production fusion test compilation failed' }
    & .\build\host_motion\chassis_fusion.exe
    if ($LASTEXITCODE -ne 0) { throw 'Production fusion test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST @projectIncludes tests/chassis_motion_allocation_test.c @projectSources -lm -o build/host_motion/chassis_allocation.exe
    if ($LASTEXITCODE -ne 0) { throw 'Spin allocation compilation failed' }
    & .\build\host_motion\chassis_allocation.exe
    if ($LASTEXITCODE -ne 0) { throw 'Spin allocation test failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST @projectIncludes tests/chassis_motion_heading_test.c @projectSources -lm -o build/host_motion/chassis_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading integration compilation failed' }
    & .\build\host_motion\chassis_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading integration test failed' }
    # 瞬时无延迟轮速用例显式关闭提前量；真实延迟由下面的独立响应模型覆盖。
    & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST '-DMC_SPIN_PHASE_LEAD_S=0.0f' @projectIncludes tests/chassis_motion_spin_test.c @projectSources -lm -o build/host_motion/chassis_spin.exe
    if ($LASTEXITCODE -ne 0) { throw 'Spin integration compilation failed' }
    & .\build\host_motion\chassis_spin.exe
    if ($LASTEXITCODE -ne 0) { throw 'Fixed-heading spin integration test failed' }
    foreach ($lead in '0.0f', '0.0135f') {
        & gcc -std=c11 -Wall -Wextra -Werror -O2 -DMC_HOST_TEST "-DMC_SPIN_PHASE_LEAD_S=$lead" @projectIncludes tests/chassis_motion_delay_test.c @projectSources -lm -o build/host_motion/chassis_delay.exe
        if ($LASTEXITCODE -ne 0) { throw 'Delayed-wheel compilation failed' }
        & .\build\host_motion\chassis_delay.exe
        if ($LASTEXITCODE -ne 0) { throw 'Delayed-wheel direction test failed' }
    }
} finally { Pop-Location }
