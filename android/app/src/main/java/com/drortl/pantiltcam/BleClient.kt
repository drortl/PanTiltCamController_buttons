package com.drortl.pantiltcam

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import org.json.JSONObject
import java.util.UUID

/**
 * BLE link to the PanTiltCam controller (see setupBle()/handleBleCommand() in
 * the firmware's src/main.cpp). Scans for the device, connects, writes text
 * commands to the command characteristic and polls the JSON status
 * characteristic. Reconnects on its own while [wantConnected] is true.
 *
 * Android allows only one GATT operation at a time, so all reads and writes
 * go through a queue. Everything runs on the main thread; GATT callbacks are
 * posted to it. Callers must hold the Bluetooth permissions first.
 */
@SuppressLint("MissingPermission")
class BleClient(private val ctx: Context, private val listener: Listener) {

    interface Listener {
        fun onConnectionState(text: String, connected: Boolean)
        fun onStatus(status: JSONObject)
    }

    companion object {
        // Must match BLE_*_UUID / BLE_DEVICE_NAME in the firmware's include/config.h.
        val SERVICE_UUID: UUID = UUID.fromString("7e1a0001-3c2b-4f5e-9a6d-1b2c3d4e5f60")
        val CMD_UUID: UUID = UUID.fromString("7e1a0002-3c2b-4f5e-9a6d-1b2c3d4e5f60")
        val STATUS_UUID: UUID = UUID.fromString("7e1a0003-3c2b-4f5e-9a6d-1b2c3d4e5f60")
        const val DEVICE_NAME = "PanTiltCam"

        private const val SCAN_TIMEOUT_MS = 15000L
        private const val POLL_MS = 500L
        private const val OP_TIMEOUT_MS = 3000L
        private const val RECONNECT_DELAY_MS = 2000L
    }

    private sealed class Op {
        class Write(val text: String) : Op()
        object Read : Op()
    }

    private val main = Handler(Looper.getMainLooper())
    private val adapter: BluetoothAdapter? =
        (ctx.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter

    private var gatt: BluetoothGatt? = null
    private var cmdChar: BluetoothGattCharacteristic? = null
    private var statusChar: BluetoothGattCharacteristic? = null
    private var scanning = false

    private val ops = ArrayDeque<Op>()
    private var current: Op? = null

    var wantConnected = false
        private set
    var connected = false
        private set

    val hasBluetooth get() = adapter != null
    val isBluetoothOn get() = adapter?.isEnabled == true

    /** True when no command or status read is waiting - used by hold-to-repeat. */
    val isIdle get() = current == null && ops.isEmpty()

    fun connect() {
        if (adapter == null) {
            listener.onConnectionState("This phone has no Bluetooth", false)
            return
        }
        wantConnected = true
        startScan()
    }

    fun disconnect() {
        wantConnected = false
        stopScan()
        main.removeCallbacksAndMessages(null)
        closeGatt()
        listener.onConnectionState("Disconnected", false)
    }

    fun send(text: String) {
        if (!connected) return
        ops.addLast(Op.Write(text))
        runNext()
    }

    // ---------- Scan ----------

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            main.post {
                if (!scanning) return@post
                stopScan()
                listener.onConnectionState("Connecting...", false)
                gatt = result.device.connectGatt(ctx, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            main.post {
                scanning = false
                listener.onConnectionState("Scan failed (error $errorCode) - retrying...", false)
                scheduleReconnect()
            }
        }
    }

    private val scanTimeout = Runnable {
        if (scanning) {
            stopScan()
            listener.onConnectionState("$DEVICE_NAME not found - retrying...", false)
            scheduleReconnect()
        }
    }

    private fun startScan() {
        if (scanning) return
        val scanner = adapter?.bluetoothLeScanner
        if (scanner == null || !isBluetoothOn) {
            listener.onConnectionState("Bluetooth is off", false)
            return
        }
        closeGatt()
        // Match by service UUID or by name - either one is enough.
        val filters = listOf(
            ScanFilter.Builder().setServiceUuid(ParcelUuid(SERVICE_UUID)).build(),
            ScanFilter.Builder().setDeviceName(DEVICE_NAME).build()
        )
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
        scanning = true
        scanner.startScan(filters, settings, scanCallback)
        listener.onConnectionState("Searching for $DEVICE_NAME...", false)
        main.postDelayed(scanTimeout, SCAN_TIMEOUT_MS)
    }

    private fun stopScan() {
        main.removeCallbacks(scanTimeout)
        if (!scanning) return
        scanning = false
        try {
            adapter?.bluetoothLeScanner?.stopScan(scanCallback)
        } catch (_: Exception) {
        }
    }

    private fun scheduleReconnect() {
        if (!wantConnected) return
        main.postDelayed({
            if (wantConnected && !connected && !scanning && gatt == null) startScan()
        }, RECONNECT_DELAY_MS)
    }

    // ---------- GATT ----------

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            main.post {
                if (g != gatt) return@post
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    listener.onConnectionState("Connected, reading services...", false)
                    // Bigger packets so the WiFi settings command fits in one
                    // write; services are discovered once the MTU is set.
                    if (!g.requestMtu(247)) g.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    closeGatt()
                    listener.onConnectionState(
                        if (wantConnected) "Connection lost - reconnecting..." else "Disconnected", false
                    )
                    scheduleReconnect()
                }
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
            main.post { if (g == gatt) g.discoverServices() }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            main.post {
                if (g != gatt) return@post
                val svc = g.getService(SERVICE_UUID)
                cmdChar = svc?.getCharacteristic(CMD_UUID)
                statusChar = svc?.getCharacteristic(STATUS_UUID)
                if (cmdChar == null || statusChar == null) {
                    closeGatt()
                    listener.onConnectionState("Wrong device (no $DEVICE_NAME service) - retrying...", false)
                    scheduleReconnect()
                    return@post
                }
                connected = true
                listener.onConnectionState("Connected to ${g.device.name ?: DEVICE_NAME}", true)
                main.post(poll)
            }
        }

        // Android 12 and older.
        @Deprecated("Deprecated in Java")
        override fun onCharacteristicRead(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            if (Build.VERSION.SDK_INT >= 33) return
            @Suppress("DEPRECATION")
            val value = c.value ?: ByteArray(0)
            onReadDone(g, value, status)
        }

        // Android 13 and newer.
        override fun onCharacteristicRead(
            g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray, status: Int
        ) {
            onReadDone(g, value, status)
        }

        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            main.post { if (g == gatt) opDone() }
        }
    }

    private fun onReadDone(g: BluetoothGatt, value: ByteArray, status: Int) {
        val text = String(value, Charsets.UTF_8)
        main.post {
            if (g != gatt) return@post
            opDone()
            if (status == BluetoothGatt.GATT_SUCCESS) {
                try {
                    listener.onStatus(JSONObject(text))
                } catch (_: Exception) {
                }
            }
        }
    }

    // ---------- Operation queue ----------

    private val poll = object : Runnable {
        override fun run() {
            if (!connected) return
            if (current !is Op.Read && ops.none { it is Op.Read }) {
                ops.addLast(Op.Read)
                runNext()
            }
            main.postDelayed(this, POLL_MS)
        }
    }

    // A lost callback must not block the queue forever.
    private val opTimeout = Runnable {
        current = null
        runNext()
    }

    private fun runNext() {
        if (current != null) return
        val g = gatt ?: return
        while (ops.isNotEmpty()) {
            val op = ops.removeFirst()
            val started = when (op) {
                is Op.Read -> statusChar?.let { g.readCharacteristic(it) } ?: false
                is Op.Write -> cmdChar?.let { write(g, it, op.text.toByteArray(Charsets.UTF_8)) } ?: false
            }
            if (started) {
                current = op
                main.postDelayed(opTimeout, OP_TIMEOUT_MS)
                return
            }
        }
    }

    private fun opDone() {
        main.removeCallbacks(opTimeout)
        current = null
        runNext()
    }

    @Suppress("DEPRECATION")
    private fun write(g: BluetoothGatt, c: BluetoothGattCharacteristic, bytes: ByteArray): Boolean {
        return if (Build.VERSION.SDK_INT >= 33) {
            g.writeCharacteristic(c, bytes, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) ==
                BluetoothStatusCodes.SUCCESS
        } else {
            c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            c.value = bytes
            g.writeCharacteristic(c)
        }
    }

    private fun closeGatt() {
        connected = false
        main.removeCallbacks(poll)
        main.removeCallbacks(opTimeout)
        ops.clear()
        current = null
        cmdChar = null
        statusChar = null
        gatt?.let {
            try {
                it.disconnect()
                it.close()
            } catch (_: Exception) {
            }
        }
        gatt = null
    }
}
