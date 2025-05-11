import cv2 as cv

def get_color_mask(hsv_img, hsv_bounds):
    """
    Create a binary mask for a color, supporting single or multiple HSV ranges.
    hsv_bounds can be a tuple (lower, upper) or a list of tuples [(lower1, upper1), (lower2, upper2)]
    """
    if isinstance(hsv_bounds, list):
        masks = [cv.inRange(hsv_img, low, high) for (low, high) in hsv_bounds]
        return cv.bitwise_or(masks[0], masks[1]) if len(masks) == 2 else masks[0]
    else:
        return cv.inRange(hsv_img, hsv_bounds[0], hsv_bounds[1])