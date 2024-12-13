import time
from pyb import I2C # type: ignore 

class BNO055:
    # BNO055 default I2C address and register addresses
    BNO055_I2C_ADDR = 0x28
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_CALIB_STAT_ADDR = 0x35
    BNO055_EULER_H_LSB_ADDR = 0x1A
    BNO055_GYRO_DATA_X_LSB_ADDR = 0x14
    BNO055_CALIB_DATA_ADDR = 0x55 # start of the calibration data addresses

    # Operation modes
    CONFIG_MODE = 0x00

    # Fusion modes
    IMU_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C  
    

    def __init__(self, i2c_bus):
        # Initialize I2C connection
        self.i2c = I2C(i2c_bus, I2C.MASTER)
        self.i2c.init(I2C.MASTER, baudrate=400000)
        self._set_mode(self.NDOF_MODE)
        #while not self.get_calibration_status():
        #   print(self.get_calibration_status_values())
        #   time.sleep(0.05)
        #   pass

    def _write_byte(self, register, value):
        """Write a byte to the specified register."""
        self.i2c.mem_write(value, self.BNO055_I2C_ADDR, register)

    def _read_byte(self, register):
        """Read a byte from the specified register."""
        return self.i2c.mem_read(1, self.BNO055_I2C_ADDR, register)[0]

    def _set_mode(self, mode):
        """Set the operation mode of the BNO055."""
        self._write_byte(self.BNO055_OPR_MODE_ADDR, mode)
        time.sleep(0.05)  # Delay to allow mode switch

    def get_calibration_status(self):
        """Get the calibration status of the sensor."""
        calib_status = self._read_byte(self.BNO055_CALIB_STAT_ADDR) & 0xFF
        if calib_status==0xFF:
            return True
        else:
            return False
    
    def get_calibration_status_values(self):
        """Get the calibration status of the sensor."""
        calib_status = self._read_byte(self.BNO055_CALIB_STAT_ADDR)
        sys = (calib_status >> 6) & 0x03
        gyro = (calib_status >> 4) & 0x03
        accel = (calib_status >> 2) & 0x03
        mag = calib_status & 0x03
        return {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}

    def write_calibration_data(self, data):     # Assumes data array is in the right format
        """Write calibration data to the sensor."""
        self._set_mode(self.CONFIG_MODE)
        for i, byte in enumerate(data):
            self._write_byte(self.BNO055_CALIB_DATA_ADDR + i, byte)
        self._set_mode(self.NDOF_MODE)

    def read_calibration_data(self):
        """Read calibration data from the sensor."""
        self._set_mode(self.CONFIG_MODE)
        data = [self._read_byte(self.BNO055_CALIB_DATA_ADDR + i) for i in range(22)]
        self._set_mode(self.NDOF_MODE)
        return data

    def get_euler_angles(self):
        """Get Euler angles (heading, roll, pitch) in degrees."""
        raw_data = self.i2c.mem_read(6, self.BNO055_I2C_ADDR, self.BNO055_EULER_H_LSB_ADDR)
        heading = (raw_data[1] << 8 | raw_data[0])
        if heading >= 0x8000:  # correcting for signed value
            heading -= 0x10000
        heading = heading / 16.0
        roll = (raw_data[3] << 8 | raw_data[2])
        if roll >= 0x8000:  # correcting for signed value
            roll -= 0x10000
        roll = roll / 16.0
        pitch = (raw_data[5] << 8 | raw_data[4])
        if pitch >= 0x8000:  # correcting for signed value
            pitch -= 0x10000
        pitch = pitch / 16.0
        return {'heading': heading, 'roll': roll, 'pitch': pitch}
    
    def get_heading(self):
        """Get the heading (yaw) angle in degrees."""
        # Read 2 bytes for the heading (yaw) angle
        raw_data = self.i2c.mem_read(2, self.BNO055_I2C_ADDR, self.BNO055_EULER_H_LSB_ADDR)
        heading = (raw_data[1] << 8 | raw_data[0])
        if heading >= 0x8000:  # correcting for signed value
            heading -= 0x10000
        heading = heading / 16.0
        return heading

    def get_angular_velocity(self):
        """Get angular velocity (x, y, z) in degrees per second."""
        raw_data = self.i2c.mem_read(6, self.BNO055_I2C_ADDR, self.BNO055_GYRO_DATA_X_LSB_ADDR)
        x = (raw_data[1] << 8 | raw_data[0])
        if x >= 0x8000: # correcting for signed value
            x-=0x10000
        x = x / 16.0
        y = (raw_data[3] << 8 | raw_data[2])
        if y >= 0x8000: # correcting for signed value
            y-=0x10000
        y = y / 16.0
        z = (raw_data[5] << 8 | raw_data[4])
        if z >= 0x8000: # correcting for signed value
            z-=0x10000
        z = z / 16.0
        return {'x': x, 'y': y, 'z': z}
    
