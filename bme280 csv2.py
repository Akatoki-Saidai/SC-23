from smbus2 import SMBus
import time
from make_csv import make_csv  # make_csv関数をインポート

class BME280:
    bus_number  = 1  # I2Cバス番号
    i2c_address = 0x76  # BME280のI2Cアドレス
    
    bus = smbus.SMBus(bus_number)  # I2Cバスの初期化
    
    digT = []  # 温度キャリブレーションデータ
    digP = []  # 気圧キャリブレーションデータ
    digH = []  # 湿度キャリブレーションデータ
    
    t_fine = 0.0  # 温度補正用の変数
    
    # レジスタにデータを書き込む関数
    def writeReg(self, reg_address, data):
        self.bus.write_byte_data(self.i2c_address, reg_address, data)
    
    # キャリブレーションデータを取得する関数
    def get_calib_param(self):
        calib = []
        for i in range(0x88, 0x88 + 24):
            calib.append(self.bus.read_byte_data(self.i2c_address, i))
        calib.append(self.bus.read_byte_data(self.i2c_address, 0xA1))
        for i in range(0xE1, 0xE1 + 7):
            calib.append(self.bus.read_byte_data(self.i2c_address, i))
    
        # キャリブレーションデータをリストに格納
       [1] << 8) | calib[0])
        self.digT.append((calib[3] << 8) | calib[2])
        self.digT.append((calib[5] << 8) | calib[4])
        self.digP.append((calib[7] << 8) | calib[6])
        self.digP.append((calib[9] << 8) | calib[8])
        self.digP.append((calib[11] << 8) | calib[10])
        self.digP.append((calib[13] << 8) | calib[12])
        self.digP.append((calib[15] << 8) | calib[14])
        self.digP.append((calib[17] << 8) | calib[16])
        self.digP.append((calib[19] << 8) | calib[18])
        self.digP.append((calib[21] << 8) | calib[20])
        self.digP.append((calib[23] << 8) | calib[22])
        self.digH.append(calib[24])
        self.digH.append((calib[26] << 8) | calib[25])
        self.digH.append(calib[27])
        self.digH.append((calib[28] << 4) | (0x0F & calib[29]))
        self.digH.append((calib[30] << 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append(calib[31])
    
    # 温度を読み取る関数
    def read_temperature(self):
        data = []
        for i in range(0xF7, 0xF7 + 8):
            data.append(self.bus.read_byte_data(self.i2c_address, i))
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return self.compensate_T(temp_raw)
    
    # 気圧を読み取る関数
    def read_pressure(self):
        data = []
        for i in range(0xF7, 0xF7 + 8):
            data.append(self.bus.read_byte_data(self.i2c_address, i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return self.compensate_P(pres_raw)
    
    def compensate_P(self, adc_P):
        global t_fine
        pressure = 0.0
        v1 = (self.t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768
    
               pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)
    
        print("pressure : %7.2f hPa" % (pressure / 100))
        return pressure / 100
    
    def compensate_T(self, adc_T):
        global t_fine
        v1 = (adc_T / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (adc_T / 131072.0 - self.digT[0] / 8192.0) * (adc_T / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        self.t_fine = v1 + v2
        temperature = self.t_fine / 5120.0
        print("temp : %-6.2f ℃" % (temperature))
        return temperature

    def get_altitude(self, qnh=1013.25, manual_temperature=None):
        # qnh = pressure at sea level where the readings are being taken.
        # The temperature should be the outdoor temperature.
        # Use the manual_temperature variable if temperature adjustments are required.
        temperature, pressure = self.compensate_P()
        if manual_temperature is not None:
            temperature = manual_temperature
        # altitude = ((pow((qnh / pressure), (1.0 / 5.257)) - 1) * (temperature + 273.15)) / 0.0065
        altitude = (((1 - (pow((pressure / qnh), 0.190284))) * 145366.45) / 0.3048 ) / 10
        print('alt', altitude)
        return altitude
        
    def get_baseline(self):
        baseline_values = []
        baseline_size = 100
    
        for i in range(baseline_size):
            _, pressure = self.compensate_P()
            baseline_values.append(pressure)
            time.sleep(0.1)
        baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])
        print('alt_base_press', baseline)
        return baseline
    
    # センサーの設定を行う関数
    def setup(self):
        osrs_t = 1  # 温度オーバーサンプリング設定
        osrs_p = 1  # 気圧オーバーサンプリング設定
        osrs_h = 1  # 湿度オーバーサンプリング設定
        mode = 3    # センサーモード（通常モード）
        t_sb = 5    # スタンバイ時間
        filter = 0  # フィルター設定
        spi3w_en = 0  # SPI 3ワイヤーモード無効化

        # 各レジスタに設定を書き込む
        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg = osrs_h

        self.writeReg(0xF2, ctrl_hum_reg)
        self.writeReg(0xF4, ctrl_meas_reg)
        self.writeReg(0xF5, config_reg)
    
    # センサーの初期設定とキャリブレーションデータの取得
    setup()
    get_calib_param()

if __name__ == '__main__':
    try:
		    readData()
    
    try:
        while True:
            temperature = bme280.read_temperature()  # 温度を読み取る
            pressure = bme280.read_pressure()  # 気圧を読み取る
            data = [temperature, pressure]
            make_csv('bme280_data.csv', data)  # データをCSVに書き込む
            time.sleep(1)  # 1秒待機
           pass  # プログラム終了時の処理
