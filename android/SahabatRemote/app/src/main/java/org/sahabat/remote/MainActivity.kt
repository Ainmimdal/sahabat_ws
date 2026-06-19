package org.sahabat.remote

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.InputDevice
import android.view.KeyEvent
import android.view.MotionEvent
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.delay

class MainActivity : ComponentActivity(), android.hardware.input.InputManager.InputDeviceListener {
    private lateinit var client: OperatorClient
    @Volatile private var deadman = false
    @Volatile private var axisX = 0f
    @Volatile private var axisY = 0f
    private val handler = Handler(Looper.getMainLooper())
    private val publish = object : Runnable {
        override fun run() {
            client.teleop(if (deadman) -axisY * .20f else 0f, if (deadman) -axisX * .60f else 0f, deadman)
            handler.postDelayed(this, 100)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        client = OperatorClient(this)
        getSystemService(android.hardware.input.InputManager::class.java).registerInputDeviceListener(this, handler)
        setContent { SahabatScreen(client) }
    }
    override fun onResume() { super.onResume(); handler.post(publish) }
    override fun onStop() { handler.removeCallbacks(publish); client.estop("Android backgrounded"); super.onStop() }
    override fun onDestroy() {
        getSystemService(android.hardware.input.InputManager::class.java).unregisterInputDeviceListener(this)
        client.close(); super.onDestroy()
    }
    override fun onInputDeviceAdded(deviceId: Int) = Unit
    override fun onInputDeviceChanged(deviceId: Int) = Unit
    override fun onInputDeviceRemoved(deviceId: Int) {
        deadman = false; axisX = 0f; axisY = 0f; client.estop("gamepad disconnected")
    }

    override fun dispatchGenericMotionEvent(event: MotionEvent): Boolean {
        if (event.source and InputDevice.SOURCE_JOYSTICK == InputDevice.SOURCE_JOYSTICK && event.action == MotionEvent.ACTION_MOVE) {
            axisX = event.getAxisValue(MotionEvent.AXIS_X).deadzone()
            axisY = event.getAxisValue(MotionEvent.AXIS_Y).deadzone()
            return true
        }
        return super.dispatchGenericMotionEvent(event)
    }
    override fun dispatchKeyEvent(event: KeyEvent): Boolean {
        if (event.keyCode == KeyEvent.KEYCODE_BUTTON_L1) {
            deadman = event.action == KeyEvent.ACTION_DOWN
            if (!deadman) client.estop("gamepad deadman released")
            return true
        }
        return super.dispatchKeyEvent(event)
    }
    private fun Float.deadzone() = if (kotlin.math.abs(this) < .12f) 0f else this
}

@Composable private fun SahabatScreen(client: OperatorClient) {
    val status by client.status.collectAsState()
    val map by client.map.collectAsState()
    val scan by client.scan.collectAsState()
    var mapId by remember { mutableStateOf("") }
    var mapName by remember { mutableStateOf("") }
    var waypointName by remember { mutableStateOf("") }
    var token by remember { mutableStateOf("") }
    LaunchedEffect(status.connected, client.leaseId) {
        while (status.connected) { if (client.leaseId.isNotEmpty()) client.renew(); delay(2000) }
    }
    MaterialTheme(colorScheme = darkColorScheme(primary = Color(0xFF65D2C4))) {
        Row(Modifier.fillMaxSize().background(Color(0xFF101820)).padding(14.dp)) {
            Column(Modifier.weight(1f).fillMaxHeight()) {
                Text("SAHABAT MAP", color = Color(0xFF65D2C4))
                Canvas(Modifier.weight(1f).fillMaxWidth().background(Color(0xFF17242E))) {
                    drawRobotMap(map, scan, status)
                }
                Text("${status.map.ifEmpty { "No map" }} · ${status.diagnostic}", color = Color.White)
            }
            Spacer(Modifier.width(14.dp))
            Column(Modifier.width(300.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { client.estop("Android E-stop") }, colors = ButtonDefaults.buttonColors(containerColor = Color.Red), modifier = Modifier.fillMaxWidth()) { Text("E-STOP") }
                Text(if (status.connected) "Connected" else "Disconnected", color = Color.White)
                Text(if (status.estop) "Stopped" else "Ready", color = Color.White)
                if (!status.connected) {
                    OutlinedTextField(token, { token = it }, label = { Text("Operator token") })
                    Button(onClick = { client.storeToken(token); runCatching { client.connect() } }) { Text("Connect") }
                }
                Row { Button(onClick = client::acquire) { Text("Take control") }; Spacer(Modifier.width(6.dp)); Button(onClick = client::clearEstop) { Text("Clear stop") } }
                Row { Button(onClick = { client.mode("mapping") }) { Text("Map") }; Spacer(Modifier.width(6.dp)); Button(onClick = { client.mode("idle") }) { Text("Idle") } }
                OutlinedTextField(mapId, { mapId = it }, label = { Text("Map ID") })
                OutlinedTextField(mapName, { mapName = it }, label = { Text("Map name") })
                Button(onClick = { client.saveMap(mapId, mapName.ifEmpty { mapId }) }, modifier = Modifier.fillMaxWidth()) { Text("Save map") }
                Row { Button(onClick = { client.mode("localization", mapId) }) { Text("Load map") }; Spacer(Modifier.width(6.dp)); Button(onClick = { client.mode("operations", mapId) }) { Text("Gallery") } }
                OutlinedTextField(waypointName, { waypointName = it }, label = { Text("Waypoint name") })
                Button(onClick = { client.quickWaypoint(waypointName.ifEmpty { "Quick waypoint" }) }) { Text("Capture waypoint") }
                Text("Hold the gamepad left bumper to drive.", color = Color.LightGray)
            }
        }
    }
}

private fun DrawScope.drawRobotMap(map: MapFrame, scan: ScanFrame, status: RobotStatus) {
    if (map.width <= 0 || map.height <= 0) return
    val scale = minOf(size.width / map.width, size.height / map.height)
    val offsetX = (size.width - map.width * scale) / 2
    val offsetY = (size.height - map.height * scale) / 2
    val step = maxOf(1, (1f / scale).toInt())
    for (row in 0 until map.height step step) for (column in 0 until map.width step step) {
        val value = map.cells.getOrNull(row * map.width + column)?.toInt()?.and(0xff) ?: 0
        if (value > 1) drawRect(
            if (value > 60) Color(0xFFCDD5DC) else Color(0xFF253743),
            Offset(offsetX + column * scale, offsetY + (map.height - row) * scale),
            androidx.compose.ui.geometry.Size(maxOf(1f, scale * step), maxOf(1f, scale * step)),
        )
    }
    val robotX = offsetX + ((status.pose[0] - map.originX) / map.resolution).toFloat() * scale
    val robotY = offsetY + (map.height - ((status.pose[1] - map.originY) / map.resolution).toFloat()) * scale
    scan.ranges.forEachIndexed { index, range ->
        if (range > 0f && range < 12f) {
            val angle = status.pose[2].toFloat() + scan.angleMin + index * scan.angleIncrement
            drawCircle(Color(0xFFE6B94A), 1.5f, Offset(
                robotX + kotlin.math.cos(angle) * range / map.resolution * scale,
                robotY - kotlin.math.sin(angle) * range / map.resolution * scale,
            ))
        }
    }
    drawCircle(Color(0xFF65D2C4), 10f, Offset(robotX, robotY))
}
