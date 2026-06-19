package org.sahabat.remote

import android.content.Context
import android.util.Base64
import androidx.security.crypto.EncryptedSharedPreferences
import androidx.security.crypto.MasterKey
import kotlinx.coroutines.flow.MutableStateFlow
import okhttp3.CertificatePinner
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject
import java.util.UUID
import java.util.concurrent.TimeUnit

data class RobotStatus(
    val connected: Boolean = false, val estop: Boolean = true,
    val mode: Int = 0, val map: String = "", val leaseOwner: String = "",
    val diagnostic: String = "Disconnected",
    val pose: List<Double> = listOf(0.0, 0.0, 0.0),
)
data class MapFrame(
    val width: Int = 0, val height: Int = 0, val resolution: Float = .05f,
    val originX: Float = 0f, val originY: Float = 0f,
    val cells: ByteArray = byteArrayOf(),
)
data class ScanFrame(
    val angleMin: Float = 0f, val angleIncrement: Float = 0f,
    val ranges: List<Float> = emptyList(),
)

class OperatorClient(context: Context) : WebSocketListener() {
    val status = MutableStateFlow(RobotStatus())
    val map = MutableStateFlow(MapFrame())
    val scan = MutableStateFlow(ScanFrame())
    val clientId = "android-${UUID.randomUUID().toString().take(8)}"
    @Volatile var leaseId = ""
    @Volatile private var sequence = 0
    private val preferences = EncryptedSharedPreferences.create(
        context, "operator",
        MasterKey.Builder(context).setKeyScheme(MasterKey.KeyScheme.AES256_GCM).build(),
        EncryptedSharedPreferences.PrefKeyEncryptionScheme.AES256_SIV,
        EncryptedSharedPreferences.PrefValueEncryptionScheme.AES256_GCM,
    )
    private var socket: WebSocket? = null

    fun connect() {
        val token = preferences.getString("token", "") ?: ""
        require(token.length >= 32) { "Set the operator token before connecting" }
        require(BuildConfig.CERT_SHA256.startsWith("sha256/")) { "Set CERT_SHA256 in app/build.gradle.kts" }
        val client = OkHttpClient.Builder().pingInterval(2, TimeUnit.SECONDS)
            .certificatePinner(CertificatePinner.Builder().add(BuildConfig.ROBOT_HOST, BuildConfig.CERT_SHA256).build()).build()
        val request = Request.Builder().url("wss://${BuildConfig.ROBOT_HOST}:8443/operator")
            .header("Authorization", "Bearer $token").build()
        socket = client.newWebSocket(request, this)
    }

    fun storeToken(token: String) { preferences.edit().putString("token", token).apply() }
    fun send(command: JSONObject) { socket?.send(command.toString()) }
    fun acquire() = send(JSONObject().put("command", "acquire").put("client_id", clientId))
    fun renew() = send(JSONObject().put("command", "renew").put("client_id", clientId).put("lease_id", leaseId))
    fun estop(reason: String) {
        teleop(0f, 0f, false)
        send(JSONObject().put("command", "estop").put("active", true).put("confirmation", reason).put("lease_id", leaseId))
    }
    fun clearEstop() = send(JSONObject().put("command", "estop").put("active", false).put("confirmation", "CLEAR").put("lease_id", leaseId))
    fun mode(mode: String, map: String = "") = send(JSONObject().put("command", "mode").put("mode", mode).put("map_id", map).put("lease_id", leaseId))
    fun saveMap(id: String, name: String) = send(JSONObject().put("command", "save_map").put("map_id", id).put("display_name", name).put("editable", true).put("overwrite", false).put("lease_id", leaseId))
    fun quickWaypoint(name: String) = send(JSONObject().put("command", "quick_waypoint").put("name", name).put("lease_id", leaseId))
    fun teleop(linear: Float, angular: Float, held: Boolean) {
        sequence += 1
        send(JSONObject().put("command", "teleop").put("client_id", clientId).put("lease_id", leaseId)
            .put("sequence", sequence).put("deadman", held).put("linear", linear).put("angular", angular))
    }
    override fun onOpen(webSocket: WebSocket, response: Response) {
        status.value = status.value.copy(connected = true, diagnostic = "Connected")
    }
    override fun onMessage(webSocket: WebSocket, text: String) {
        val data = JSONObject(text)
        if (data.optString("type") == "status") status.value = RobotStatus(
            true, data.optBoolean("estop", true), data.optInt("mode"), data.optString("active_map"),
            data.optString("lease_owner"), data.optString("diagnostic"),
            data.optJSONArray("pose")?.let { listOf(it.optDouble(0), it.optDouble(1), it.optDouble(2)) }
                ?: listOf(0.0, 0.0, 0.0),
        )
        if (data.optString("type") == "map") {
            val origin = data.getJSONArray("origin")
            map.value = MapFrame(
                data.getInt("width"), data.getInt("height"), data.getDouble("resolution").toFloat(),
                origin.getDouble(0).toFloat(), origin.getDouble(1).toFloat(),
                Base64.decode(data.getString("cells"), Base64.DEFAULT),
            )
        }
        if (data.optString("type") == "scan") {
            val values = data.getJSONArray("ranges")
            scan.value = ScanFrame(
                data.getDouble("angle_min").toFloat(), data.getDouble("angle_increment").toFloat(),
                List(values.length()) { values.optDouble(it).toFloat() },
            )
        }
        if (data.optString("command") in listOf("acquire", "renew") && data.optBoolean("granted")) {
            leaseId = data.optString("lease_id")
        }
    }
    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
        status.value = RobotStatus(diagnostic = t.message ?: "Connection lost")
    }
    fun close() { estop("app closing"); socket?.close(1000, "closing") }
}
