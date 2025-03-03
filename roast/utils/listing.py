def compare_tuple(actual: tuple, reference: tuple):
    """Compare Two lists"""
    for element in actual:
        if element not in reference:
            return False

    return True
