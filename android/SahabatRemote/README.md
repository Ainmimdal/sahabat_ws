# Sahabat Android mapping remote

This optional Android 10+ app is a thin client for the robot's TLS mobile
gateway. It stores its bearer token with Android Keystore-backed encrypted
preferences, pins the gateway certificate, and treats backgrounding, socket
loss and left-bumper release as stop conditions.

Before building, set `ROBOT_HOST` and `CERT_SHA256` in `app/build.gradle.kts`.
Provision a gateway token of at least 32 random characters with
`OperatorClient.storeToken`; do not commit the token. This workspace currently
has no Android SDK or Gradle wrapper, so build and signing must be performed in
Android Studio.

The map view renders the occupancy grid, lidar returns and robot pose. It is an
operator aid, not a substitute for the spotter's direct clearance check.
