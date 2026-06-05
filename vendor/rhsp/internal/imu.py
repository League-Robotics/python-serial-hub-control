
"""
IMU constants and register definitions for BNO055 sensor.
"""

class IMUConstants:
    """Constants for BNO055 IMU sensor register addresses and configuration values."""
    
    # Device configuration
    IMU_ADDRESS = 40
    
    # Register addresses - Device information
    PAGE_ID = 7
    CHIP_ID = 0
    ACC_ID = 1
    MAG_ID = 2
    GYR_ID = 3
    SW_REV_ID_LSB = 4
    SW_REV_ID_MSB = 5
    BL_REV_ID = 6
    
    # Data registers - Accelerometer
    ACC_DATA_X_LSB = 8
    ACC_DATA_X_MSB = 9
    ACC_DATA_Y_LSB = 10
    ACC_DATA_Y_MSB = 11
    ACC_DATA_Z_LSB = 12
    ACC_DATA_Z_MSB = 13
    
    # Data registers - Magnetometer
    MAG_DATA_X_LSB = 14
    MAG_DATA_X_MSB = 15
    MAG_DATA_Y_LSB = 16
    MAG_DATA_Y_MSB = 17
    MAG_DATA_Z_LSB = 18
    MAG_DATA_Z_MSB = 19
    
    # Data registers - Gyroscope
    GYR_DATA_X_LSB = 20
    GYR_DATA_X_MSB = 21
    GYR_DATA_Y_LSB = 22
    GYR_DATA_Y_MSB = 23
    GYR_DATA_Z_LSB = 24
    GYR_DATA_Z_MSB = 25
    
    # Data registers - Euler angles
    EUL_H_LSB = 26
    EUL_H_MSB = 27
    EUL_R_LSB = 28
    EUL_R_MSB = 29
    EUL_P_LSB = 30
    EUL_P_MSB = 31
    
    # Data registers - Quaternion
    QUA_DATA_W_LSB = 32
    QUA_DATA_W_MSB = 33
    QUA_DATA_X_LSB = 34
    QUA_DATA_X_MSB = 35
    QUA_DATA_Y_LSB = 36
    QUA_DATA_Y_MSB = 37
    QUA_DATA_Z_LSB = 38
    QUA_DATA_Z_MSB = 39
    
    # Data registers - Linear acceleration
    LIA_DATA_X_LSB = 40
    LIA_DATA_X_MSB = 41
    LIA_DATA_Y_LSB = 42
    LIA_DATA_Y_MSB = 43
    LIA_DATA_Z_LSB = 44
    LIA_DATA_Z_MSB = 45
    
    # Data registers - Gravity vector
    GRV_DATA_X_LSB = 46
    GRV_DATA_X_MSB = 47
    GRV_DATA_Y_LSB = 48
    GRV_DATA_Y_MSB = 49
    GRV_DATA_Z_LSB = 50
    GRV_DATA_Z_MSB = 51
    
    # Status and control registers
    TEMP = 52
    CALIB_STAT = 53
    SELFTEST_RESULT = 54
    INTR_STAT = 55
    SYS_CLK_STAT = 56
    SYS_STAT = 57
    SYS_ERR = 58
    UNIT_SEL = 59
    DATA_SELECT = 60
    OPR_MODE = 61
    PWR_MODE = 62
    SYS_TRIGGER = 63
    TEMP_SOURCE = 64
    
    # Operation modes
    CONFIGMODE = 0
    ACCONLY = 1
    MAGONLY = 2
    GYROONLY = 3
    ACCMAG = 4
    ACCGYRO = 5
    MAGGYRO = 6
    AMG = 7
    IMUMODE = 8
    COMPASS = 9
    M4G = 10
    NDOF_FMC_OFF = 11
    NDOF = 12
    
    # Power modes
    NORMAL = 0
    LOW_POWER = 1
    SUSPEND = 2
    
    # Axis configuration
    AXIS_MAP_CONFIG = 65
    AXIS_MAP_SIGN = 66
    
    # Soft iron calibration matrix
    SIC_MATRIX_0_LSB = 67
    SIC_MATRIX_0_MSB = 68
    SIC_MATRIX_1_LSB = 69
    SIC_MATRIX_1_MSB = 70
    SIC_MATRIX_2_LSB = 71
    SIC_MATRIX_2_MSB = 72
    SIC_MATRIX_3_LSB = 73
    SIC_MATRIX_3_MSB = 74
    SIC_MATRIX_4_LSB = 75
    SIC_MATRIX_4_MSB = 76
    SIC_MATRIX_5_LSB = 77
    SIC_MATRIX_5_MSB = 78
    SIC_MATRIX_6_LSB = 79
    SIC_MATRIX_6_MSB = 80
    SIC_MATRIX_7_LSB = 81
    SIC_MATRIX_7_MSB = 82
    SIC_MATRIX_8_LSB = 83
    SIC_MATRIX_8_MSB = 84
    
    # Offset registers - Accelerometer
    ACC_OFFSET_X_LSB = 85
    ACC_OFFSET_X_MSB = 86
    ACC_OFFSET_Y_LSB = 87
    ACC_OFFSET_Y_MSB = 88
    ACC_OFFSET_Z_LSB = 89
    ACC_OFFSET_Z_MSB = 90
    
    # Offset registers - Magnetometer
    MAG_OFFSET_X_LSB = 91
    MAG_OFFSET_X_MSB = 92
    MAG_OFFSET_Y_LSB = 93
    MAG_OFFSET_Y_MSB = 94
    MAG_OFFSET_Z_LSB = 95
    MAG_OFFSET_Z_MSB = 96
    
    # Offset registers - Gyroscope
    GYR_OFFSET_X_LSB = 97
    GYR_OFFSET_X_MSB = 98
    GYR_OFFSET_Y_LSB = 99
    GYR_OFFSET_Y_MSB = 100
    GYR_OFFSET_Z_LSB = 101
    GYR_OFFSET_Z_MSB = 102
    
    # Radius registers
    ACC_RADIUS_LSB = 103
    ACC_RADIUS_MSB = 104
    MAG_RADIUS_LSB = 105
    MAG_RADIUS_MSB = 106
    
    # Unit selection constants
    EUL_UNIT_DEG = 0
    EUL_UNIT_RAD = 4
    GYR_UNIT_DPS = 0
    GYR_UNIT_RPS = 2
    ACC_UNIT_MSS = 0
    ACC_UNIT_MG = 1
    
    # Configuration registers (Page 1)
    ACC_CONFIG = 8
    MAG_CONFIG = 9
    GYR_CONFIG_0 = 10
    GYR_CONFIG_1 = 11
    ACC_SLEEP_CONFIG = 12
    GYR_SLEEP_CONFIG = 13
    INT_MSK = 15
    INT_EN = 16
    ACC_AM_THRES = 17
    ACC_INT_SETTINGS = 18
    ACC_HG_DURATION = 19
    ACC_HG_THRES = 20
    ACC_NM_THRES = 21
    ACC_NM_SET = 22
    GRYO_INT_SETTING = 23
    GRYO_HR_X_SET = 24
    GRYO_DUR_ = 25
    GRYO_HR_Y_SET = 26
    GRYO_DUR_Y = 27
    GRYO_HR_Z_SET = 28
    GRYO_DUR_Z = 29
    GRYO_AM_THRES = 30
    GRYO_AM_SET = 31
    
    # Unique ID registers
    UNIQUE_ID_FIRST = 80
    UNIQUE_ID_LAST = 95


