import pyquaternion as pyq
import math

# Create a hypothetical orientation of the upper leg and lower leg
# We use the (axis, degrees) notation because it's the most intuitive here
# Upper leg perfectly vertical with a slight rotation
q_upper = pyq.Quaternion(axis=[0.0, 0.0, -1.0], degrees=-5)
# Lower leg a little off-vertical, with a rotation in the other direction.
q_lower = pyq.Quaternion(axis=[0.1, -0.2, -0.975], degrees=10)

# Get the 3D difference between these two orientations
qd = q_upper.conjugate * q_lower

# Calculate Euler angles from this difference quaternion
phi   = math.atan2( 2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x**2 + qd.y**2) )
theta = math.asin ( 2 * (qd.w * qd.y - qd.z * qd.x) )
psi   = math.atan2( 2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y**2 + qd.z**2) )
print("phi->%s theta->%s psi->%s" % (phi, theta, psi))