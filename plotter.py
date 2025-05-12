import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import numpy as np

def plot_motion_data(csv_file):
    # Чтение данных из CSV
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Ошибка: файл {csv_file} не найден")
        return
    except Exception as e:
        print(f"Ошибка при чтении файла: {e}")
        return

    # Проверка наличия необходимых столбцов
    required_columns = ['time', 'com_x', 'com_y', 'com_z', 'zmp_x', 'zmp_y']
    if not all(col in df.columns for col in required_columns):
        print("Ошибка: в файле отсутствуют необходимые столбцы данных")
        print(f"Найдены столбцы: {list(df.columns)}")
        print(f"Ожидаемые столбцы: {required_columns}")
        return

    # 1. Традиционные 2D графики
    plt.figure(figsize=(15, 10))
    
    # График X координат
    plt.subplot(3, 1, 1)
    plt.plot(df['time'], df['com_x'], label='CoM X', color='blue')
    plt.plot(df['time'], df['zmp_x'], label='ZMP X', color='red', linestyle='--')
    plt.xlabel('Время (с)')
    plt.ylabel('Положение X (м)')
    plt.title('Сравнение CoM и ZMP по осям')
    plt.grid(True)
    plt.legend()

    # График Y координат
    plt.subplot(3, 1, 2)
    plt.plot(df['time'], df['com_y'], label='CoM Y', color='green')
    plt.plot(df['time'], df['zmp_y'], label='ZMP Y', color='orange', linestyle='--')
    plt.xlabel('Время (с)')
    plt.ylabel('Положение Y (м)')
    plt.grid(True)
    plt.legend()

    # График Z координаты
    plt.subplot(3, 1, 3)
    plt.plot(df['time'], df['com_z'], label='CoM Z', color='purple')
    plt.xlabel('Время (с)')
    plt.ylabel('Высота Z (м)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

    # 2. 3D траектория движения CoM и ZMP
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Траектория CoM
    ax.plot(df['com_x'], df['com_y'], df['com_z'], 
            label='Траектория CoM', color='blue', linewidth=2)
    
    # Траектория ZMP (проекция на плоскость)
    ax.plot(df['zmp_x'], df['zmp_y'], np.zeros(len(df['zmp_x'])), 
            label='Траектория ZMP (проекция)', color='red', linestyle='--')
    
    # Соединительные линии между CoM и ZMP (для наглядности)
    for i in range(0, len(df), 10):  # Берем каждую 10-ю точку для читаемости
        ax.plot([df['com_x'].iloc[i], df['zmp_x'].iloc[i]],
                [df['com_y'].iloc[i], df['zmp_y'].iloc[i]],
                [df['com_z'].iloc[i], 0],
                color='gray', alpha=0.3, linewidth=0.5)

    ax.set_xlabel('Ось X (м)')
    ax.set_ylabel('Ось Y (м)')
    ax.set_zlabel('Ось Z (м)')
    ax.set_title('3D траектория движения центра масс (CoM) и центра давления (ZMP)')
    ax.legend()
    
    # Добавим плоскость для ZMP
    xx, yy = np.meshgrid(np.linspace(df['com_x'].min()-0.1, df['com_x'].max()+0.1, 10),
                         np.linspace(df['com_y'].min()-0.1, df['com_y'].max()+0.1, 10))
    zz = np.zeros(xx.shape)
    ax.plot_surface(xx, yy, zz, alpha=0.2, color='gray')
    
    plt.tight_layout()
    plt.show()

    # 3. Фазовый портрет (производные координат)
    if 'com_x_vel' not in df.columns:
        # Вычисляем скорости если их нет в данных
        dt = np.diff(df['time'])
        df['com_x_vel'] = np.gradient(df['com_x'], df['time'])
        df['com_y_vel'] = np.gradient(df['com_y'], df['time'])
        df['com_z_vel'] = np.gradient(df['com_z'], df['time'])

    plt.figure(figsize=(14, 5))
    
    # Фазовый портрет по X
    plt.subplot(1, 2, 1)
    plt.plot(df['com_x'], df['com_x_vel'], label='Фазовый портрет X', color='blue')
    plt.xlabel('Положение X (м)')
    plt.ylabel('Скорость X (м/с)')
    plt.title('Фазовый портрет движения по X')
    plt.grid(True)
    
    # Фазовый портрет по Y
    plt.subplot(1, 2, 2)
    plt.plot(df['com_y'], df['com_y_vel'], label='Фазовый портрет Y', color='green')
    plt.xlabel('Положение Y (м)')
    plt.ylabel('Скорость Y (м/с)')
    plt.title('Фазовый портрет движения по Y')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Визуализация данных движения робота')
    parser.add_argument('--file', type=str, default='motion_data.csv',
                       help='Путь к CSV файлу с данными (по умолчанию: motion_data.csv)')
    
    args = parser.parse_args()
    plot_motion_data(args.file)