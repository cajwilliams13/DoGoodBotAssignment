import os
from datetime import datetime
from math import floor, log10


def plural(single, count, plural_form=None):
    """Pluralise word by adding s"""
    plural_form = (single + 's') if plural_form is None else plural_form
    return single if count == 1 else plural_form


def safe_write_to_file(filename, data, new_file=False):
    directory = os.path.dirname(filename)
    if directory and not os.path.exists(directory):
        os.makedirs(directory)
        
    if new_file and os.path.exists(filename):
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        filename = f"{filename}_{timestamp}"

    with open(filename, 'w') as f:
        f.write(data)
    return filename


def eng_unit(value, unit):
    """Convert a value into a more convenient SI unit (0.005 m -> 5 mm)"""

    unit_prefix = ('n', 'u', 'm', '', 'k', 'M')  # Assumes unit is SI-compatible

    if value == 0:
        return f"0 {unit}"

    exponent_value = int(floor(log10(abs(value))))
    index = (exponent_value + 9) // 3

    if index < 0:
        index = 0                  # Set to 'nano' if smaller
    elif index >= len(unit_prefix):
        index = len(unit_prefix) - 1  # Set to the largest available unit if bigger

    new_value = value * (10 ** (-index * 3 + 9))
    new_unit = unit_prefix[index] + unit

    if int(new_value) == new_value:
        return f"{int(new_value)} {new_unit}"
    return f"{round(new_value, 2)} {new_unit}"


def lerp(a, b, fac):
    return a * fac + b * (1-fac)
