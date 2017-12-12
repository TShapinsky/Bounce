# Bounce
Bounce is computer-vision-controlled stewart platform designed to juggle ping pong balls. Algorithms predict where the ball will fall, our stewart platform moves into place, and our laser sensing system triggers our hitting mechanism to fire.

## Dependencies
- `opencv-2`
- `python2`
- `scipy`
- `numpy`
- `bitstring`

## Running
``python2 track_omega.py``

### Arguments
- `--csv` log ball locations to `points.csv`
- `--serial` send desired servo positions over serial (required to demo)
- `--headless` run without webcam output frames
- `--predict` use predictive modeling to move platform under ball

### Tweaking for different Stewart platforms
The `findangles.py` file contains variables for defining the different parameters of a stewart platform, including servo min and max values.
