object_measures = (2, 1, 2)
object_coords = (0, 0, 0)

# Cada dedo se define como una tupla:
# (x_inicial, y_inicial, z_inicial, offset_x, offset_y, offset_z)
fingers = [
    (0, 0, 0, -0.5,  0,  1),
    (0, 0, 0,  0.5, 0.5,  0.5),
    (0, 0, 0,  0.5, 0.5, -0.5)
]

def calculate_center(coords, measures):
    """Calcula el centro del objeto dado el vértice inferior y las medidas."""
    return tuple(c + m/2 for c, m in zip(coords, measures))

def calculate_final_finger_position(center, finger):
    """
    Calcula la posición final de un dedo a partir del centro del objeto
    y el offset (almacenado en las posiciones 3, 4 y 5 de la tupla 'finger').
    """
    offset = finger[3:6]
    final_pos = tuple(c + o for c, o in zip(center, offset))
    # Retornamos la posición final junto con el offset para referencia
    return final_pos + offset

def calculate_finger_positions(center, fingers):
    """Calcula las posiciones finales para todos los dedos dados un centro."""
    return [calculate_final_finger_position(center, finger) for finger in fingers]

def print_finger_positions(label, finger_positions):
    print(label)
    for i, pos in enumerate(finger_positions, start=1):
        print(f"Finger {i}: {pos}")

def main():
    # Usando el centro calculado a partir del objeto
    center = calculate_center(object_coords, object_measures)
    positions_center = calculate_finger_positions(center, fingers)
    print_finger_positions("Posiciones de dedos usando el centro del objeto:", positions_center)

    # Ejemplo: usar otro centro arbitrario
    new_center = (3, 0.5, 1)
    positions_new = calculate_finger_positions(new_center, fingers)
    print_finger_positions("Posiciones de dedos usando nuevo centro:", positions_new)

if __name__ == '__main__':
    main()