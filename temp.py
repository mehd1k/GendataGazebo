import numpy as np

def remove_close_elements(arr, threshold):
    """
    Remove elements from the array that are within the threshold distance
    from any other element.

    :param arr: Input array of integers.
    :param threshold: Threshold distance.
    :return: Array with close elements removed.
    """
    result = []
    for elem in arr:
        if all(abs(elem - x) >= threshold for x in result):
            result.append(elem)
    return np.array(result)

# Example usage
arr = np.array([1, 3, 4, 7, 10, 11, 14])
threshold = 2
filtered_arr = remove_close_elements(arr, threshold)
print(filtered_arr)
