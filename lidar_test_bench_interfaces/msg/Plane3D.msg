# Unit normal vector of the plane
float32 normal_x
float32 normal_y
float32 normal_z

# Centroid of the fitted points on the plane
float32 centroid_x
float32 centroid_y
float32 centroid_z

# Signed distance from origin along normal (dot(normal, centroid))
float32 distance

# RMS point-to-plane residual (0.0 for profile/expected planes)
float32 rms_error
