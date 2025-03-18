import sys
import logging
from PySide6.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
                               QPushButton, QStackedWidget)
from PySide6.QtCore import Qt, Signal, QThread, QTimer
from PySide6.QtGui import QPixmap
import serial
import time
import numpy as np
import struct
import glob

# Настройка логирования
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

Commands = {
    'GET_SPECTRUM_FLOAT': 0xC2
}

def fletcher_checksum(data):
    CK_A = 0
    CK_B = 0
    for byte in data:
        CK_A = (CK_A + byte) % 256
        CK_B = (CK_B + CK_A) % 256
    return CK_A, CK_B

def parse_response(buffer):
    try:
        if len(buffer) < 6:
            logging.warning(f"Buffer too short to parse: {len(buffer)} bytes")
            return None, None

        start_byte = buffer[0]
        if start_byte != 0xBB:
            logging.warning(f"Invalid start byte: {start_byte:#02x}")
            return None, None

        cmd = buffer[1]
        if cmd != Commands['GET_SPECTRUM_FLOAT']:
            logging.warning(f"Incorrect command code: {cmd:#02x}")
            return None, None

        payload_len = struct.unpack('<H', buffer[2:4])[0]
        if len(buffer) < 6 + payload_len:
            logging.warning(f"Incomplete packet: expected {6 + payload_len} bytes, got {len(buffer)}")
            return None, None

        received_crc = struct.unpack('<H', buffer[4 + payload_len:6 + payload_len])[0]
        calc_crc1, calc_crc2 = fletcher_checksum(buffer[:4 + payload_len])
        calculated_crc = (calc_crc2 << 8) | calc_crc1

        if received_crc != calculated_crc:
            logging.warning(f"CRC mismatch: received {received_crc:#06x}, calculated {calculated_crc:#06x}")
            return None, None

        data = buffer[4:4 + payload_len]
        num_floats = len(data) // 4

        if num_floats % 2 != 0:
            logging.warning(f"Incorrect number of floats: {num_floats}")
            return None, None

        float_data = struct.unpack(f'<{num_floats}f', data)
        mags = np.array(float_data[:num_floats // 2])
        freqs = np.array(float_data[num_floats // 2:]) / 1e6  # Конвертация в MHz

        # Логирование значений частот для проверки
        logging.debug(f"Parsed frequencies (MHz): {freqs}")

        return mags, freqs
    except Exception as e:
        logging.error(f"Error parsing response: {e}")
        return None, None

def get_spectrum(serial_conn, start_freq=800e6, stop_freq=4800e6):
    command = struct.pack(
        '<BBHQQBBB',
        0xBB,
        Commands['GET_SPECTRUM_FLOAT'],
        19,
        int(start_freq),
        int(stop_freq),
        2, 3, 0
    )
    crc1, crc2 = fletcher_checksum(command)
    command += struct.pack('<BB', crc1, crc2)
    logging.debug(f"Sent GET_SPECTRUM_FLOAT command: {command.hex()}")
    serial_conn.write(command)

class SpectrumWorker(QThread):
    data_updated = Signal(list, list)
    status_changed = Signal(str)
    alert_triggered = Signal(float, float)
    raw_data_received = Signal(bytes)

    def __init__(self):
        super().__init__()
        self.running = False
        self.threshold = -97  # Порог для включения светодиода
        self.port = '/dev/ttyACM0'  # Порт основного устройства
        self.baudrate = 115200
        self.buffer = bytearray()
        self.last_alert_time = 0
        self.min_alert_interval = 1

        # Настройки Arduino
        self.arduino_port = '/dev/ttyACM1'  # ЗАМЕНИТЕ НА ВАШ ПОРТ!
        self.arduino_baudrate = 9600
        self.arduino_conn = None

    def connect_to_device(self):
        try:
            self.status_changed.emit("Поиск устройства...")
            available_ports = self.list_available_ports()
            if not available_ports:
                self.status_changed.emit("Устройство не найдено")
                return False

            self.port = available_ports[0]
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=2
            )
            self.status_changed.emit("Устройство подключено")
            logging.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            self.status_changed.emit(f"Ошибка подключения: {e}")
            logging.error(f"Serial connection error: {e}")
            return False

    def connect_arduino(self):
        try:
            if self.arduino_conn is not None:
                self.arduino_conn.close()
                self.arduino_conn.dtr = False  # Отключаем автоматический сброс
                self.arduino_conn.rts = False

            self.arduino_conn = serial.Serial(
                port=self.arduino_port,
                baudrate=self.arduino_baudrate,
                timeout=1
            )
            logging.info(f"Connected to Arduino at {self.arduino_port}")
            return True
        except Exception as e:
            logging.error(f"Arduino connection error: {e}")
            self.arduino_conn = None
            return False

    def send_to_arduino(self, command):
        if self.arduino_conn and self.arduino_conn.is_open:
            try:
                self.arduino_conn.write(command.encode())
                logging.debug(f"Sent to Arduino: {command}")
            except Exception as e:
                logging.error(f"Error sending to Arduino: {e}")

    def list_available_ports(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def run(self):
        while not self.connect_to_device():
            time.sleep(5)

        # Подключение к Arduino
        if not self.connect_arduino():
            self.status_changed.emit("Ошибка подключения к Arduino")

        self.running = True
        try:
            while self.running:
                if not self.ser or not self.ser.is_open:
                    self.status_changed.emit("Устройство отключено. Попытка переподключения...")
                    while not self.connect_to_device():
                        time.sleep(5)
                    get_spectrum(self.ser)

                data = self.ser.read(1024)
                if data:
                    self.raw_data_received.emit(data)
                    self.buffer.extend(data)
                    while True:
                        start_idx = self.buffer.find(b'\xBB')
                        if start_idx == -1:
                            break
                        self.buffer = self.buffer[start_idx:]
                        if len(self.buffer) < 4:
                            break
                        payload_len = struct.unpack('<H', self.buffer[2:4])[0]
                        packet_len = 6 + payload_len
                        if len(self.buffer) < packet_len:
                            break
                        packet = self.buffer[:packet_len]
                        self.buffer = self.buffer[packet_len:]

                        mags, freqs = parse_response(packet)
                        if mags is not None and freqs is not None:
                            current_time = time.time()
                            has_peak = False
                            for amp, freq in zip(mags, freqs):
                                if amp > self.threshold:
                                    has_peak = True
                                    logging.info(f"Peak detected: {amp} dBm @ {freq} MHz")
                                    if (current_time - self.last_alert_time) > self.min_alert_interval:
                                        self.alert_triggered.emit(amp, freq)
                                        self.last_alert_time = current_time

                            # Управление светодиодом
                            if has_peak:
                                self.send_to_arduino('1')
                            else:
                                self.send_to_arduino('0')

                            self.data_updated.emit(freqs.tolist(), mags.tolist())
        except Exception as e:
            self.buffer.clear()
            self.status_changed.emit(f"Ошибка: {e}")
            logging.error(f"An error occurred: {e}")
        finally:
            if self.ser:
                self.ser.close()
            if self.arduino_conn:
                self.arduino_conn.close()
            self.running = False
            logging.info("Ports closed")
            self.status_changed.emit("Устройства отключены")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_worker()

    def init_ui(self):
        self.setWindowTitle("Анализатор спектра с управлением LED")
        self.setGeometry(100, 100, 800, 600)
        central_widget = QWidget()
        layout = QVBoxLayout()

        self.status_label = QLabel("Статус: Не подключено")
        self.alert_label = QLabel("Последнее событие: Нет")
        self.battery_label = QLabel("Уровень заряда батареи: 75%")
        self.warning_label = QLabel("")
        self.warning_label.setStyleSheet("QLabel { color : red; font-weight: bold; }")
        self.warning_label.setVisible(False)

        self.sound_button = QPushButton()
        self.sound_button.setIcon(QPixmap("images/on_sound.png"))
        self.sound_button.setIconSize(self.sound_button.sizeHint())
        self.sound_button.clicked.connect(self.toggle_sound)

        self.enable_button = QPushButton("Вкл")
        self.enable_button.setStyleSheet("QPushButton { background-color : #66CC66; color : white; }")
        self.enable_button.clicked.connect(self.toggle_enable)

        self.map_button = QPushButton()
        self.map_button.setIcon(QPixmap("images/map.png"))
        self.map_button.setIconSize(self.map_button.sizeHint())
        self.map_button.clicked.connect(lambda: self.log_event("Переход на карту"))

        self.settings_button = QPushButton()
        self.settings_button.setIcon(QPixmap("images/setting.png"))
        self.settings_button.setIconSize(self.settings_button.sizeHint())
        self.settings_button.clicked.connect(lambda: self.log_event("Переход к настройкам"))

        self.logs_button = QPushButton()
        self.logs_button.setIcon(QPixmap("images/log1.png"))
        self.logs_button.setIconSize(self.logs_button.sizeHint())
        self.logs_button.clicked.connect(lambda: self.log_event("Переход к логам"))

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.map_button)
        button_layout.addWidget(self.settings_button)
        button_layout.addWidget(self.logs_button)

        layout.addWidget(QLabel("Система 5TC", alignment=Qt.AlignCenter))
        layout.addWidget(self.status_label)
        layout.addWidget(self.alert_label)
        layout.addWidget(self.battery_label)
        layout.addWidget(self.warning_label)
        layout.addWidget(self.sound_button)
        layout.addWidget(self.enable_button)
        layout.addLayout(button_layout)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def init_worker(self):
        self.worker = SpectrumWorker()
        self.worker.status_changed.connect(self.update_status)
        self.worker.alert_triggered.connect(self.show_alert)
        self.worker.raw_data_received.connect(self.handle_raw_data)

        self.timer = QTimer()
        self.timer.timeout.connect(self.request_spectrum)
        self.timer.setInterval(1000)

        self.worker.start()
        self.timer.start()

    def update_status(self, message):
        self.status_label.setText(f"Статус: {message}")

    def show_alert(self, amp, freq):
        self.alert_label.setText(f"Последнее событие: {amp:.1f} dBm @ {freq:.2f} MHz")
        self.warning_label.setText(f"Внимание! Обнаружен сигнал: {amp:.1f} dBm @ {freq:.2f} MHz")
        self.warning_label.setVisible(True)

    def handle_raw_data(self, data):
        logging.debug(f"Received raw data: {data.hex()}")

    def request_spectrum(self):
        if self.worker.isRunning():
            get_spectrum(self.worker.ser)

    def toggle_sound(self):
        sound_on = not self.sound_button.property("soundOn")
        self.sound_button.setProperty("soundOn", sound_on)
        icon = QPixmap("images/on_sound.png" if sound_on else "images/off_sound.png")
        self.sound_button.setIcon(icon)
        self.sound_button.setIconSize(self.sound_button.sizeHint())
        # Добавьте логику для включения/выключения звука

    def toggle_enable(self):
        enable_on = not self.enable_button.property("enableOn")
        self.enable_button.setProperty("enableOn", enable_on)
        self.enable_button.setText("Выкл" if enable_on else "Вкл")
        self.enable_button.setStyleSheet("QPushButton { background-color : #FF7373; color : white; }" if enable_on else "QPushButton { background-color : #66CC66; color : white; }")
        # Добавьте логику для включения/выключения устройства

    def log_event(self, event):
        logging.info(f"Log event: {event}")
        # Добавьте логику для логирования события

    def closeEvent(self, event):
        # Останавливаем поток безопасно
        if self.worker.isRunning():
            self.worker.running = False
            self.worker.wait(2000)  # Даем 2 секунды на завершение

        # Закрываем соединения с проверкой
        try:
            if self.worker.arduino_conn is not None:
                self.worker.arduino_conn.close()
        except Exception as e:
            logging.error(f"Error closing Arduino: {e}")

        try:
            if self.worker.ser is not None:
                self.worker.ser.close()
        except Exception as e:
            logging.error(f"Error closing main device: {e}")

        # Обязательно вызываем родительский метод
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
