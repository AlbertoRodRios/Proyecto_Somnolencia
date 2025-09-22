import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Configuración de estilo
plt.style.use('default')
sns.set_palette("tab10")

# Cargar datos desde archivo CSV
df = pd.read_csv('/home/dastl/Documentos/Development stuff/Proyecto_Somnolencia/features.csv')

# Verificar la carga de datos
print("Dimensiones del dataset:", df.shape)
print("\nNombres de columnas:")
print(df.columns.tolist())

# Crear visualizaciones con más subplots
fig = plt.figure(figsize=(20, 20))

# 1. Características del acelerómetro (6 subplots)
features_acc = ['rms', 'var', 'energy', 'ptp', 'skew', 'kurt']
for i, feat in enumerate(features_acc, 1):
    plt.subplot(6, 3, i)
    for axis in ['ax', 'ay', 'az']:
        plt.plot(df[f'{axis}_{feat}'], label=axis.upper(), marker='o', markersize=3)
    plt.title(f'Acelerómetro - {feat.upper()}')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlabel('Muestras')

# 2. Características del giroscopio (6 subplots)
features_gyro = ['rms', 'var', 'energy', 'ptp', 'skew', 'kurt']
for i, feat in enumerate(features_gyro, 7):
    plt.subplot(6, 3, i)
    for axis in ['gx', 'gy', 'gz']:
        plt.plot(df[f'{axis}_{feat}'], label=axis.upper(), marker='o', markersize=3)
    plt.title(f'Giroscopio - {feat.upper()}')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlabel('Muestras')

# 3. Correlaciones cruzadas (6 subplots)
xcorr_features = [col for col in df.columns if 'xcorr' in col]
for i, feat in enumerate(xcorr_features, 13):
    plt.subplot(6, 3, i)
    plt.plot(df[feat], marker='o', markersize=3)
    plt.title(f'Correlación: {feat[6:].upper()}')
    plt.grid(True, alpha=0.3)
    plt.xlabel('Muestras')

plt.tight_layout()
plt.savefig('analisis_sensores_completo.png', dpi=300)
plt.show()

# Gráficas adicionales de energía y RMS
fig, axes = plt.subplots(2, 2, figsize=(15, 10))

# Energía de acelerómetro
for axis in ['ax', 'ay', 'az']:
    axes[0, 0].plot(df[f'{axis}_energy'], label=f'{axis.upper()}', linewidth=2, marker='o', markersize=3)
axes[0, 0].set_title('Energía - Acelerómetro')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)
axes[0, 0].set_xlabel('Muestras')

# Energía de giroscopio
for axis in ['gx', 'gy', 'gz']:
    axes[0, 1].plot(df[f'{axis}_energy'], label=f'{axis.upper()}', linewidth=2, marker='o', markersize=3)
axes[0, 1].set_title('Energía - Giroscopio')
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)
axes[0, 1].set_xlabel('Muestras')

# RMS comparativo
for axis in ['ax', 'ay', 'az']:
    axes[1, 0].plot(df[f'{axis}_rms'], label=f'{axis.upper()}', linewidth=2, marker='o', markersize=3)
axes[1, 0].set_title('RMS - Acelerómetro')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)
axes[1, 0].set_xlabel('Muestras')

for axis in ['gx', 'gy', 'gz']:
    axes[1, 1].plot(df[f'{axis}_rms'], label=f'{axis.upper()}', linewidth=2, marker='o', markersize=3)
axes[1, 1].set_title('RMS - Giroscopio')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)
axes[1, 1].set_xlabel('Muestras')

plt.tight_layout()
plt.savefig('energia_rms_comparativo.png', dpi=300)
plt.show()

# Análisis de valores estadísticos
print("\nEstadísticas descriptivas:")
print(df.describe())

# Matriz de correlación (solo para algunas variables clave para evitar sobrecarga)
variables_clave = ['ax_rms', 'ay_rms', 'az_rms', 'gx_rms', 'gy_rms', 'gz_rms', 
                   'ax_energy', 'ay_energy', 'az_energy', 'gx_energy', 'gy_energy', 'gz_energy']
corr_matrix = df[variables_clave].corr()

plt.figure(figsize=(12, 10))
sns.heatmap(corr_matrix, annot=True, cmap='coolwarm', center=0)
plt.title('Matriz de Correlación entre Variables Clave')
plt.tight_layout()
plt.savefig('matriz_correlacion.png', dpi=300)
plt.show()