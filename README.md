Написание спектрального анализатор.
ПО получает данные с устройства, через tcp порт. Выводит график, строит автоматические пороги для диапазонов за счет медианной, экспоненты среднего показатели и максимального значения. Также интерфейс имеет возможность отображать график, редактировать, УДАЛЯТЬ И Добавлять диапазон.
Есть кнопка отладки , через n минут строит автопороги.
Список библиотек

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
