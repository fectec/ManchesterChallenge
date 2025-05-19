# ColorBlobDetection.msg
# Message describing a detected color blob

# Color constants (uint8):
uint8 COLOR_NONE   = 0  # No blob detected
uint8 COLOR_GREEN  = 1  # Green blob: Continue with the path
uint8 COLOR_YELLOW = 2  # Yellow blob: Drive slowly, until you see a red light to stop
uint8 COLOR_RED    = 3  # Red blob: Stop until you see a green light

# Current detection data:
uint8   color           # One of the COLOR_* constants above