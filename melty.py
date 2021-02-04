import analogio
import board
import time
 
 
# Create analog inputs for each ADXL335 axis.
# x_axis = analogio.AnalogIn(board.A1)
# y_axis = analogio.AnalogIn(board.A2)
z_axis = analogio.AnalogIn(board.A2)
 
# Define function to convert raw analog values to gravities.
def accel_value(axis):
    # Convert axis value to float within 0...1 range.
    val = axis.value / 65535
    # Shift values to true center (0.5).
    val -= 0.5
    # Convert to gravities.
    return val * 3.0
 
# Main loop prints acceleration every second.
while True:
    # x = accel_value(x_axis)
    # y = accel_value(y_axis)
    z = accel_value(z_axis)
    # print('Acceleration (G): ({0})'.format(z))
    print(z_axis.value)
    time.sleep(.1)