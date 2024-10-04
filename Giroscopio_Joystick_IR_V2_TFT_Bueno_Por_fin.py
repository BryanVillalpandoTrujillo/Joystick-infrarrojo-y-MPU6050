from machine import SPI,Pin, ADC, SoftI2C
import time
import ssd1306
import ir_rx
from imu import MPU6050
from ST7735 import TFT
from sysfont import sysfont
import math

tft_CS = 5
tft_RESET = 4
tft_A0 = 2
tft_SDA = 23
tft_SCK = 18

# Configurar SPI
spi = SPI(2, baudrate=20000000, polarity=0, phase=0, miso=None)

# Inicializar la pantalla TFT ST7735
tft = TFT(spi, tft_A0, tft_RESET, tft_CS)
tft.initr()
tft.rgb(True)


tft_ancho = 128
tft_alto = 160

# Icono matriz
ICONO = [
    [0, 0, 1, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 1, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0],
    [1, 1, 0, 0, 1, 0, 0, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [0, 0, 1, 1, 0, 1, 1, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 0],
]

# Tamaño inicial del ícono
tamaño = 1
escala_min = 1
escala_max = 5

# Inicializar el joystick
vrx = ADC(Pin(12, Pin.IN))
vry = ADC(Pin(13, Pin.IN))
vrx.atten(ADC.ATTN_11DB)
vry.atten(ADC.ATTN_11DB)

# Inicializar el MPU6050
i2c_mpu = SoftI2C(scl=Pin(22), sda=Pin(21))
mpu_address = 0
mpu = MPU6050(i2c_mpu, mpu_address)

# Diccionario de códigos IR
ir_key = {
    0x19: "Arriba",  # Código IR para "Arriba"
    0x18: "Abajo",   # Código IR para "Abajo"
}

#M
def Map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def leer_suavizado(adc, muestras=10):
    total = 0
    for _ in range(muestras):
        total += adc.read()
        time.sleep_ms(1)
    return total // muestras


def dibujar_icono(x_pos, y_pos, tamaño):
    tft.fill(TFT.BLACK)  # Limpia la pantalla antes de dibujar
    for y, fila in enumerate(ICONO):
        for x, c in enumerate(fila):
            if c:  # Solo dibujar si el valor del ícono es 1
                for dy in range(tamaño):
                    for dx in range(tamaño):
                        tft.pixel((x * tamaño + dx + x_pos, y * tamaño + dy + y_pos), TFT.CYAN)

def recalcular_posiciones(tamaño):
    """Recalcula las posiciones centradas en la pantalla para el ícono, dado un tamaño."""
    icono_ancho = len(ICONO[0]) * tamaño  # Ancho del ícono escalado
    icono_alto = len(ICONO) * tamaño      # Alto del ícono escalado
    x_pos = (tft_ancho - icono_ancho) // 2  # Centrar el ícono en el eje X
    y_pos = (tft_alto - icono_alto) // 2    # Centrar el ícono en el eje Y
    return x_pos, y_pos

def callback(data, addr, ctrl):
    print("Código recibido:", hex(data), "Dirección:", hex(addr), "Control:", hex(ctrl))  # Imprime el código y sus atributos
    global tamaño
    if data == 0x19:  # "Arriba"
        if tamaño < escala_max:
            tamaño += 1
            print("Creciendo ícono, tamaño:", tamaño)
    elif data == 0x18:  # "Abajo"
        if tamaño > escala_min:
            tamaño -= 1
            print("Decreciendo ícono, tamaño:", tamaño)
    
    # Recalcular posiciones y redibujar el ícono
    x_pos, y_pos = recalcular_posiciones(tamaño)
    dibujar_icono(x_pos, y_pos, tamaño)

# Inicialización del receptor IR usando el protocolo NEC16
ir = ir_rx.NEC_16(Pin(26, Pin.IN), callback)

# Zona muerta para el joystick
zona_muerta = 100   # Ajusta el valor según la sensibilidad que deseas
x_pos = tft_ancho // 2 - 4  # Centrar el ícono en el eje X
y_pos = tft_alto // 2 - 4   # Centrar el ícono en el eje Y

while True:
    # Leer valores del joystick
    x_joy = leer_suavizado(vrx)
    y_joy = leer_suavizado(vry)

    # Calibrar los valores mínimos y máximos del joystick
    x_min, x_max = 500, 4095
    y_min, y_max = 500, 4095

    # Si el valor está fuera de la zona muerta, cambiar la posición
    if abs(x_joy - (x_min + x_max) // 2) > zona_muerta:
        # Invertir la dirección del joystick si es necesario
        x_pos = int((x_max - x_joy) * (tft_ancho - tamaño) / (x_max - x_min))  # Ajustar el tamaño

    if abs(y_joy - (y_min + y_max) // 2) > zona_muerta:
        # Invertir la dirección del joystick si es necesario
        y_pos = int((y_max - y_joy) * (tft_alto - tamaño) / (y_max - y_min))  # Ajustar el tamaño
            
    # Leer aceleración y giroscopio del MPU6050
    aceleracion = mpu.accel
    giroscopio = mpu.gyro

    # Actualizar la posición del ícono basada en el giroscopio
    x_pos += int(giroscopio.x * 0.5)  # ajustar el factor
    y_pos += int(giroscopio.y *0.5 )  # ajustar el factor

    # Limitar la posición dentro de los límites de la pantalla TFT
    # Limitar la posición dentro de los límites de la pantalla TFT
    icono_ancho = len(ICONO[0]) * tamaño  # Ancho del ícono escalado
    icono_alto = len(ICONO) * tamaño      # Alto del ícono escalado

    # Ajustar límites de movimiento basados en el tamaño del ícono
    x_pos = max(0, min(x_pos, tft_ancho - icono_ancho))
    y_pos = max(0, min(y_pos, tft_alto - icono_alto))
    
    # Dibujar el ícono en la nueva posición
    dibujar_icono(x_pos, y_pos, tamaño)
    
    # Pausa breve antes de la siguiente lectura
    time.sleep(0.1)

