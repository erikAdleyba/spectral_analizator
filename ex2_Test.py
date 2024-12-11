import struct
import serial
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import logging
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import FuncFormatter, MultipleLocator
from scipy.ndimage import median_filter
from scipy.signal import find_peaks
import threading
import time

logging.basicConfig(
    filename='spectrum_log.txt',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
TIMEOUT = 10

Commands = {
    'GET_SPECTRUM_FLOAT': 0xC2,
    'SET_GENERATOR_POINT': 0xC3
}

def fletcher_checksum(data):
    CK_A = 0
    CK_B = 0
    for byte in data:
        CK_A = (CK_A + byte) % 256
        CK_B = (CK_B + CK_A) % 256
    return CK_A, CK_B

def parse_response(serial_conn):
    try:
        header = serial_conn.read(4)
        if len(header) < 4:
            return None, None

        payload_len = struct.unpack('<H', header[2:4])[0]
        response_data = serial_conn.read(payload_len)
        if len(response_data) < payload_len:
            return None, None

        received_crc = serial_conn.read(2)
        calc_crc1, calc_crc2 = fletcher_checksum(header + response_data)
        if received_crc != bytes([calc_crc1, calc_crc2]):
            return None, None

        float_data = struct.unpack(f'<{payload_len // 4}f', response_data)
        mags = np.array(float_data[:len(float_data) // 2])
        freqs = np.array(float_data[len(float_data) // 2:]) / 1e6  # Преобразование в МГц
        return mags, freqs
    except serial.SerialException:
        return None, None

def get_spectrum_float(serial_conn, start_freq, stop_freq, rfin=2, bw=3, speed=0):
    command = struct.pack(
        '<BBHQQBBB',
        0xBB,
        Commands['GET_SPECTRUM_FLOAT'],
        19,
        start_freq,
        stop_freq,
        rfin, bw, speed
    )
    crc1, crc2 = fletcher_checksum(command)
    command += struct.pack('BB', crc1, crc2)

    try:
        serial_conn.write(command)
        response = parse_response(serial_conn)
        return response
    except serial.SerialException:
        return None, None

class SpectrumAnalyzerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Spectrum Analyzer")

        self.screen_width = root.winfo_screenwidth()
        self.screen_height = root.winfo_screenheight()
        self.root.geometry(f"{self.screen_width}x{self.screen_height}")

        self.scan_ranges = [
            (400000000, 500000000, None),
            #  (800000000, 1150000000, None),
            # (1150000000, 1500000000, None),
            # (1500000000, 1850000000, None),
        ]
        self.alert_flags = [{'high': False} for _ in self.scan_ranges]
        self.ignored_ranges = set()
        self.persistent_signals = set()

        # Диапазон частот для отображения
        self.display_range = (200000000,1300000000)

        main_frame = tk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        control_frame = tk.Frame(main_frame, width=220)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        self.range_listbox = tk.Listbox(control_frame, width=30, height=12)
        self.range_listbox.pack(fill=tk.BOTH, padx=5, pady=5)

        add_button = tk.Button(control_frame, text="Добавить диапазон", command=self.add_range)
        edit_button = tk.Button(control_frame, text="Редактировать диапазон", command=self.edit_range)
        delete_button = tk.Button(control_frame, text="Удалить диапазон", command=self.delete_range)
        start_calibration_button = tk.Button(control_frame, text="Пуск наладочного периода", command=self.start_calibration)

        log_button = tk.Button(control_frame, text="Посмотреть логи", command=self.show_logs)
        log_button.pack(fill=tk.X, padx=5, pady=2)

        add_button.pack(fill=tk.X, padx=5, pady=2)
        edit_button.pack(fill=tk.X, padx=5, pady=2)
        delete_button.pack(fill=tk.X, padx=5, pady=2)
        start_calibration_button.pack(fill=tk.X, padx=5, pady=2)

        self.threshold_entry = tk.Entry(control_frame, width=10)
        self.threshold_entry.pack(pady=(0, 10))
        self.threshold_entry.insert(0, "-110")

        self.canvas_frame = tk.Frame(main_frame)
        self.canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(12, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.serial_conn = None
        self.connect_to_device()

        self.ema_alpha = 0.1  # Начальное значение коэффициента сглаживания
        self.ema_values = [None] * len(self.scan_ranges)  # Хранение текущих значений EMA для каждого диапазона

        self.hysteresis_threshold = 0.5  # Гистерезис в дБм
        self.stability_duration = 3  # Время стабильности в секундах
        self.stability_counter = [0] * len(self.scan_ranges)

        self.auto_calibrate_thresholds()

        self.update_range_listbox()
        self.update_spectrum()

    def connect_to_device(self):
        try:
            self.serial_conn = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось подключиться к порту {PORT}: {e}")
            self.serial_conn = None

    def reconnect_device(self):
        if self.serial_conn:
            self.serial_conn.close()
        self.connect_to_device()

    def show_logs(self):
        try:
            with open('spectrum_log.txt', 'r') as log_file:
                log_content = log_file.read()
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось прочитать файл логов: {e}")
            return

        log_window = tk.Toplevel(self.root)
        log_window.title("Логи")
        log_window.geometry(f"1200x400+{self.screen_width // 2 - 300}+{self.screen_height // 2 - 200}")

        log_text = tk.Text(log_window, wrap=tk.WORD)
        log_text.insert(tk.END, log_content)
        log_text.config(state=tk.DISABLED)
        log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        close_button = tk.Button(log_window, text="Закрыть", command=log_window.destroy)
        close_button.pack(pady=10)

    def auto_calibrate_thresholds(self):
        if self.serial_conn:
            for i, (start, stop, _) in enumerate(self.scan_ranges):
                amplitudes = []
                for _ in range(500):  # Уменьшено количество измерений
                    mags, _ = get_spectrum_float(self.serial_conn, start, stop)
                    if mags is not None:
                        amplitudes.append(mags.max())

                if amplitudes:
                    median_amplitude = np.median(amplitudes)
                    threshold = median_amplitude + 1
                    round_threshold = round(threshold, 1)
                    self.scan_ranges[i] = (start, stop, round_threshold)
                    logging.info(f"Автоустановка порога для диапазона {start}-{stop} Гц: {round_threshold} дБм")
        else:
            logging.error("Автоматическая калибровка невозможна: нет соединения с устройством.")

    def update_range_listbox(self):
        self.range_listbox.delete(0, tk.END)
        for start, stop, round_threshold in self.scan_ranges:
            self.range_listbox.insert(tk.END, f"{start/1e6:.1f} - {stop/1e6:.1f} МГц, Порог: {round_threshold} дБм")

    def process_spectrum(self, mags, freqs):
        median_value = np.median(mags)
        filtered_mags = mags[mags > median_value]
        filtered_freqs = freqs[mags > median_value]
        return filtered_mags, filtered_freqs

    def update_spectrum(self):
        if self.serial_conn:
            self.ax.clear()

            if not self.scan_ranges:
                self.ax.text(0.5, 0.5, 'Нет данных для отображения', horizontalalignment='center', verticalalignment='center', fontsize=12, color='red')
            else:
                # Отображение только одного диапазона частот
                start_freq, stop_freq = self.display_range
                mags, freqs = get_spectrum_float(self.serial_conn, start_freq, stop_freq)
                if mags is not None and freqs is not None:
                    # Медианная фильтрация
                    mags = median_filter(mags, size=3)

                    # Игнорирование известных значений
                    mags, freqs = self.ignore_known_values(mags, freqs)

                    # Обработка спектра
                    filtered_mags, filtered_freqs = self.process_spectrum(mags, freqs)

                    # Обновление EMA
                    ema_value = self.update_ema(filtered_mags.max(), 0)
                    threshold_high = ema_value + 1  # Порог на основе EMA
                    threshold_low = threshold_high - self.hysteresis_threshold  # Нижний порог гистерезиса

                    # Обнаружение пиков
                    peaks, _ = find_peaks(filtered_mags, height=threshold_high)
                    peak_freqs = filtered_freqs[peaks]
                    peak_mags = filtered_mags[peaks]

                    self.ax.plot(filtered_freqs, filtered_mags, color='blue', label='Спектр')
                    self.ax.axhline(y=threshold_high, color='red', linestyle='--', label='Порог')

                    # Отображение пиков
                    for peak_freq, peak_mag in zip(peak_freqs, peak_mags):
                        self.ax.plot(peak_freq, peak_mag, "x", color='green')
                        self.ax.annotate(f'{peak_freq:.1f} МГц\n{peak_mag:.1f} дБм',
                                         (peak_freq, peak_mag),
                                         textcoords="offset points",
                                         xytext=(5,5),
                                         ha='center')

            self.ax.set_title("Спектр")
            self.ax.set_xlabel("Частота (МГц)")
            self.ax.set_ylabel("Амплитуда (дБм)")

            # Форматирование меток на оси X
            self.ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: f'{x:.1f}'))

            # Добавление сетки
            self.ax.grid(True)

            # Улучшение меток на осях
            self.ax.xaxis.set_major_locator(MultipleLocator(50))
            self.ax.xaxis.set_minor_locator(MultipleLocator(10))
            self.ax.yaxis.set_major_locator(MultipleLocator(10))
            self.ax.yaxis.set_minor_locator(MultipleLocator(5))

            # Добавление легенды
            self.ax.legend()

            self.canvas.draw()

            # Выполнение вычислительных процессов в фоновом режиме
            self.background_processing()

        self.root.after(500, self.update_spectrum)

    def background_processing(self):
        for i, (start, stop, _) in enumerate(self.scan_ranges):
            mags, freqs = get_spectrum_float(self.serial_conn, start, stop)
            if mags is not None:
                # Медианная фильтрация
                mags = median_filter(mags, size=3)

                # Игнорирование известных значений
                mags, freqs = self.ignore_known_values(mags, freqs)

                # Обработка спектра
                filtered_mags, _ = self.process_spectrum(mags, freqs)

                # Обновление EMA
                ema_value = self.update_ema(filtered_mags.max(), i)
                threshold_high = ema_value + 1  # Порог на основе EMA
                threshold_low = threshold_high - self.hysteresis_threshold  # Нижний порог гистерезиса

                # Обнаружение пиков
                peaks, _ = find_peaks(filtered_mags, height=threshold_high)

                if filtered_mags.max() > threshold_high:
                    self.stability_counter[i] += 1
                    if self.stability_counter[i] >= self.stability_duration and not self.alert_flags[i]['high']:
                        self.alert_flags[i]['high'] = True
                        self.show_alert(f"Превышение порога в диапазоне {start/1e6}-{stop/1e6} МГц!")
                        logging.warning(f"Превышение порога в диапазоне {start/1e6}-{stop/1e6} МГц на амплитуде {filtered_mags.max():.1f} дБм!")
                elif filtered_mags.max() <= threshold_low:
                    self.alert_flags[i]['high'] = False
                    self.stability_counter[i] = 0

    def update_ema(self, current_value, index):
        if self.ema_values[index] is None:
            self.ema_values[index] = current_value
        else:
            self.ema_values[index] = self.ema_alpha * current_value + (1 - self.ema_alpha) * self.ema_values[index]
        return self.ema_values[index]

    def show_alert(self, message):
        alert = tk.Toplevel(self.root)
        alert.title("Внимание")
        alert.geometry(f"470x250+{self.screen_width // 2 - 150}+{self.screen_height // 2 - 50}")
        alert.configure(bg="red")

        alert_label = tk.Label(alert, text=message, font=("Arial", 14, "bold"), bg="red", fg="white")
        alert_label.pack(fill=tk.BOTH, expand=True, pady=10)

        close_button = tk.Button(alert, text="Закрыть", command=alert.destroy, font=("Arial", 12), bg="white", fg="red")
        close_button.pack(pady=(0, 10))

        def blink():
            current_color = alert_label.cget("foreground")
            next_color = "yellow" if current_color == "white" else "white"
            alert_label.configure(fg=next_color)
            alert.after(500, blink)

        blink()
        alert.after(5000, alert.destroy)

    def add_range(self):
        start_freq = simpledialog.askfloat("Добавить диапазон", "Начальная частота (МГц):")
        stop_freq = simpledialog.askfloat("Добавить диапазон", "Конечная частота (МГц):")
        threshold = simpledialog.askfloat("Добавить диапазон", "Порог амплитуды (дБм):")

        if start_freq is not None and stop_freq is not None and threshold is not None:
            self.scan_ranges.append((int(start_freq * 1e6), int(stop_freq * 1e6), threshold))
            self.alert_flags.append({'high': False})
            self.ema_values.append(None)  # Добавление нового значения EMA
            self.stability_counter.append(0)  # Добавление нового счетчика стабильности
            self.update_range_listbox()
            logging.info(f"Добавлен новый диапазон: {start_freq}-{stop_freq} МГц, Порог: {threshold} дБм")



    def edit_range(self):
        selected = self.range_listbox.curselection()
        if not selected:
            messagebox.showwarning("Внимание", "Выберите диапазон для редактирования.")
            return

        index = selected[0]
        start_freq = simpledialog.askfloat("Редактировать диапазон", "Начальная частота (МГц):", initialvalue=self.scan_ranges[index][0] / 1e6)
        stop_freq = simpledialog.askfloat("Редактировать диапазон", "Конечная частота (МГц):", initialvalue=self.scan_ranges[index][1] / 1e6)
        threshold = simpledialog.askfloat("Редактировать диапазон", "Порог амплитуды (дБм):", initialvalue=self.scan_ranges[index][2])

        if start_freq is not None and stop_freq is not None and threshold is not None:
            self.scan_ranges[index] = (int(start_freq * 1e6), int(stop_freq * 1e6), threshold)
            self.update_range_listbox()
            logging.info(f"Отредактирован диапазон: {start_freq}-{stop_freq} МГц, Порог: {threshold} дБм")



    def delete_range(self):
        selected = self.range_listbox.curselection()
        if not selected:
            messagebox.showwarning("Внимание", "Выберите диапазон для удаления.")
            return

        index = selected[0]
        start_freq = self.scan_ranges[index][0] / 1e6
        stop_freq = self.scan_ranges[index][1] / 1e6
        del self.scan_ranges[index]
        del self.alert_flags[index]
        del self.ema_values[index]  # Удаление значения EMA
        del self.stability_counter[index]  # Удаление счетчика стабильности
        self.update_range_listbox()
        logging.info(f"Удален диапазон: {start_freq}-{stop_freq} МГц")



    def start_calibration(self):
        self.ax.clear()
        self.ax.text(0.5, 0.5, 'Идет сканирование', horizontalalignment='center', verticalalignment='center', fontsize=12, color='red')
        self.canvas.draw()

        calibration_thread = threading.Thread(target=self.calibration_process)
        calibration_thread.start()

    def calibration_process(self):
        calibration_time = 2 * 60  # 2 минуты в секундах
        end_time = time.time() + calibration_time

        while time.time() < end_time:
            for i, (start, stop, _) in enumerate(self.scan_ranges):
                amplitudes = []
                for _ in range(10):  # Уменьшено количество измерений для оптимизации
                    mags, _ = get_spectrum_float(self.serial_conn, start, stop)
                    if mags is not None:
                        amplitudes.append(mags.max())

                if amplitudes:
                    max_amplitude = max(amplitudes)
                    threshold = max_amplitude + 1
                    round_threshold = round(threshold, 1)
                    self.scan_ranges[i] = (start, stop, round_threshold)

            time.sleep(1)  # Пауза для предотвращения перегрузки процессора

        self.update_range_listbox()
        self.update_spectrum()

    def ignore_known_values(self, mags, freqs):
        # Игнорирование значений, которые уже были зафиксированы
        filtered_mags = []
        filtered_freqs = []
        for mag, freq in zip(mags, freqs):
            if (mag, freq) not in self.ignored_ranges:
                filtered_mags.append(mag)
                filtered_freqs.append(freq)
            else:
                logging.debug(f"Игнорировано значение: {mag} дБм, {freq} МГц")
        return np.array(filtered_mags), np.array(filtered_freqs)

    def ignore_persistent_signals(self):
        # Игнорирование постоянных сигналов
        for freq in self.persistent_signals:
            self.ignored_ranges.add((freq, freq))

    def add_persistent_signal(self, freq):
        # Добавление постоянного сигнала
        self.persistent_signals.add(freq)
        self.ignore_persistent_signals()

    def remove_persistent_signal(self, freq):
        # Удаление постоянного сигнала
        if freq in self.persistent_signals:
            self.persistent_signals.remove(freq)
            self.ignore_persistent_signals()

if __name__ == "__main__":
    root = tk.Tk()
    app = SpectrumAnalyzerApp(root)
    root.mainloop()
