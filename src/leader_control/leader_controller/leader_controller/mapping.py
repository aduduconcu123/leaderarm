def clamp(value, minimum, maximum):
    """Clamp value to [minimum, maximum]."""
    return max(minimum, min(value, maximum))


def normalize_to_unit(value, minimum, maximum):
    """
    Map value from [minimum, maximum] to [-1, 1].

    Values outside the input range are clamped.
    """
    if maximum <= minimum:
        raise ValueError("maximum must be greater than minimum")

    normalized = (
        2.0 * (value - minimum) / (maximum - minimum)
        - 1.0
    )

    return clamp(normalized, -1.0, 1.0)
