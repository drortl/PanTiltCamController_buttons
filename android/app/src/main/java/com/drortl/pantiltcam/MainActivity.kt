package com.drortl.pantiltcam

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.MotionEvent
import android.view.View
import android.view.WindowManager
import android.view.inputmethod.EditorInfo
import android.view.inputmethod.InputMethodManager
import android.widget.Button
import android.widget.EditText
import android.widget.SeekBar
import android.widget.Switch
import android.widget.TextView
import android.widget.Toast
import org.json.JSONObject
import java.util.Locale

/**
 * Off-grid controller screen: same controls as the device's WiFi web page,
 * but over Bluetooth LE (see [BleClient]). Connects automatically on start.
 */
class MainActivity : Activity(), BleClient.Listener {

    companion object {
        private const val REQ_PERMS = 1
        private const val REQ_BT_ON = 2
        private const val HOLD_DELAY_MS = 400L   // hold this long before a pad button repeats
        private const val REPEAT_MS = 250L       // then send the next step this often (when the link is idle)
    }

    private lateinit var ble: BleClient
    private val main = Handler(Looper.getMainLooper())

    private lateinit var connText: TextView
    private lateinit var btnConnect: Button
    private lateinit var azText: TextView
    private lateinit var panText: TextView
    private lateinit var rawPanText: TextView
    private lateinit var tiltText: TextView
    private lateinit var azInput: EditText
    private lateinit var speedBar: SeekBar
    private lateinit var speedVal: TextView
    private lateinit var wifiSwitch: Switch
    private lateinit var wifiOnSwitch: Switch
    private lateinit var wifiNote: TextView
    private lateinit var staSsid: EditText
    private lateinit var staPass: EditText
    private lateinit var apSsid: EditText
    private lateinit var apPass: EditText
    private lateinit var panStepInput: EditText
    private lateinit var tiltStepInput: EditText
    private lateinit var limitsText: TextView
    private val presetInfo = mutableListOf<TextView>()
    private var wifiFieldsLoaded = false
    private var apName = "PanTiltCam-Setup" // updated from the status
    private lateinit var resText: TextView
    private val controlViews = mutableListOf<View>()

    private var speedTouching = false
    private var wifiSwitching = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        ble = BleClient(this, this)

        connText = findViewById(R.id.connText)
        btnConnect = findViewById(R.id.btnConnect)
        azText = findViewById(R.id.azText)
        panText = findViewById(R.id.panText)
        rawPanText = findViewById(R.id.rawPanText)
        tiltText = findViewById(R.id.tiltText)
        azInput = findViewById(R.id.azInput)
        speedBar = findViewById(R.id.speedBar)
        speedVal = findViewById(R.id.speedVal)
        wifiSwitch = findViewById(R.id.wifiSwitch)
        wifiOnSwitch = findViewById(R.id.wifiOnSwitch)
        wifiNote = findViewById(R.id.wifiNote)
        staSsid = findViewById(R.id.staSsid)
        staPass = findViewById(R.id.staPass)
        apSsid = findViewById(R.id.apSsid)
        apPass = findViewById(R.id.apPass)
        resText = findViewById(R.id.resText)

        // Main screen vs Settings screen (WiFi). Back returns to the main screen.
        findViewById<Button>(R.id.btnSettings).setOnClickListener { showSettings(true) }
        findViewById<Button>(R.id.btnBack).setOnClickListener { showSettings(false) }

        btnConnect.setOnClickListener {
            if (ble.wantConnected) ble.disconnect() else startConnect()
        }

        setupHold(R.id.btnUp, "step tilt 1")
        setupHold(R.id.btnDown, "step tilt -1")
        setupHold(R.id.btnLeft, "step pan -1")
        setupHold(R.id.btnRight, "step pan 1")
        setupClick(R.id.btnStop) { ble.send("stop") }
        setupClick(R.id.btnHome) { ble.send("home") }
        setupClick(R.id.btnSaveHome) {
            confirm("Save the current position as Home?") { ble.send("savehome") }
        }
        setupClick(R.id.btnClearHome) {
            confirm("Clear Home?") { ble.send("clearhome") }
        }
        setupClick(R.id.btnGoAz) { goAzimuth() }
        azInput.setOnEditorActionListener { _, actionId, _ ->
            if (actionId == EditorInfo.IME_ACTION_GO) { goAzimuth(); true } else false
        }
        setupClick(R.id.btnPanMid) { ble.send("panmid") }
        setupClick(R.id.btnTiltZero) { ble.send("tiltzero") }
        buildPresetRows()

        // Step sizes (Settings screen) - same values as the web page.
        panStepInput = findViewById(R.id.panStepInput)
        tiltStepInput = findViewById(R.id.tiltStepInput)
        limitsText = findViewById(R.id.limitsText)
        controlViews += listOf(panStepInput, tiltStepInput)
        setupClick(R.id.btnSaveSteps) {
            val pan = panStepInput.text.toString().toFloatOrNull()
            val tilt = tiltStepInput.text.toString().toFloatOrNull()
            if (pan == null || tilt == null || pan < 0.1f || pan > 20f || tilt < 0.1f || tilt > 20f) {
                toast("Step size must be 0.1 to 20 degrees")
            } else {
                hideKeyboard(panStepInput)
                panStepInput.clearFocus()
                tiltStepInput.clearFocus()
                ble.send("steps $pan $tilt")
                toast("Step sizes saved")
            }
        }

        controlViews += azInput
        controlViews += speedBar
        speedBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(bar: SeekBar, progress: Int, fromUser: Boolean) {
                speedVal.text = progress.toString()
            }
            override fun onStartTrackingTouch(bar: SeekBar) { speedTouching = true }
            override fun onStopTrackingTouch(bar: SeekBar) {
                speedTouching = false
                ble.send("speed ${bar.progress.coerceIn(1, 63)}")
            }
        })

        // On = Home WiFi, off = access point. The device restarts to apply it.
        controlViews += wifiSwitch
        wifiSwitch.setOnClickListener {
            val toHome = wifiSwitch.isChecked
            wifiSwitch.isChecked = !toHome // only change after confirm (status poll updates it)
            val msg = if (toHome)
                "Switch WiFi to Home network? The device restarts and joins your home WiFi " +
                    "(if not found, it starts its access point again). Bluetooth reconnects by itself."
            else
                "Switch WiFi to Access point (\"$apName\")? The device restarts. " +
                    "Bluetooth reconnects by itself."
            confirm(msg) { sendWifiCommand(if (toHome) "wifi sta" else "wifi ap") }
        }

        // WiFi on/off. Off = Bluetooth control only (Bluetooth always stays on,
        // so WiFi can always be turned back on from here).
        controlViews += wifiOnSwitch
        wifiOnSwitch.setOnClickListener {
            val turnOn = wifiOnSwitch.isChecked
            wifiOnSwitch.isChecked = !turnOn // only change after confirm (status poll updates it)
            val msg = if (turnOn)
                "Turn WiFi on? The device restarts. Bluetooth reconnects by itself."
            else
                "Turn WiFi off? The device restarts and is then controlled by Bluetooth only " +
                    "(the WiFi web page stops working). You can turn WiFi on again here."
            confirm(msg) { sendWifiCommand(if (turnOn) "wifi on" else "wifi off") }
        }

        // WiFi settings. Empty password = keep the saved one.
        controlViews += listOf(staSsid, staPass, apSsid, apPass)
        setupClick(R.id.btnSaveSta) { saveWifiConfig("sta", staSsid, staPass, "home WiFi") }
        setupClick(R.id.btnSaveAp) { saveWifiConfig("ap", apSsid, apPass, "access point") }

        setControlsEnabled(false)
        startConnect()
    }

    override fun onDestroy() {
        ble.disconnect()
        super.onDestroy()
    }

    // ---------- Connection / permissions ----------

    private fun neededPermissions(): List<String> =
        if (Build.VERSION.SDK_INT >= 31)
            listOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        else
            listOf(Manifest.permission.ACCESS_FINE_LOCATION)

    @SuppressLint("MissingPermission")
    private fun startConnect() {
        if (!ble.hasBluetooth) {
            connText.text = "This phone has no Bluetooth"
            return
        }
        val missing = neededPermissions().filter {
            checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) {
            requestPermissions(missing.toTypedArray(), REQ_PERMS)
            return
        }
        if (!ble.isBluetoothOn) {
            @Suppress("DEPRECATION")
            startActivityForResult(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE), REQ_BT_ON)
            return
        }
        ble.connect()
        btnConnect.text = "Disconnect"
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != REQ_PERMS) return
        if (grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            startConnect()
        } else {
            connText.text = "Bluetooth permission needed (Settings > Apps > PanTiltCam > Permissions > Nearby devices)"
        }
    }

    @Deprecated("Deprecated in Java")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        @Suppress("DEPRECATION")
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == REQ_BT_ON) {
            if (ble.isBluetoothOn) startConnect() else connText.text = "Bluetooth is off"
        }
    }

    // ---------- BleClient.Listener ----------

    override fun onConnectionState(text: String, connected: Boolean) {
        connText.text = text
        btnConnect.text = if (ble.wantConnected) "Disconnect" else "Connect"
        setControlsEnabled(connected)
    }

    override fun onStatus(status: JSONObject) {
        val az = status.optDouble("az")
        azText.text = if (status.optInt("azOk") == 1) fmt("Azimut: %.1f°", az) else "Azimut: --"
        if (status.optInt("home") == 0) azText.append("  (no home)")
        panText.text = if (status.optInt("pk") == 1) fmt("Pan: %.1f°", status.optDouble("pan")) else "Pan: ??"
        rawPanText.text = fmt("Motor Pan: %.1f°", status.optDouble("rp"))
        tiltText.text = fmt("Tilt: %+.1f° (0 = horizontal)", status.optDouble("rt"))

        if (!speedTouching) {
            val spd = status.optInt("spd", speedBar.progress)
            speedBar.progress = spd
            speedVal.text = spd.toString()
        }
        // Presets: [azOk, az, tiltOk, tilt] per slot, same text as the web page.
        status.optJSONArray("pr")?.let { pr ->
            for (i in 0 until minOf(pr.length(), presetInfo.size)) {
                val p = pr.optJSONArray(i) ?: continue
                val az = if (p.optInt(0) == 1) fmt("%.1f°", p.optDouble(1)) else "--"
                val tilt = if (p.optInt(2) == 1) fmt("%+.1f°", p.optDouble(3)) else "--"
                presetInfo[i].text = "Az $az, Tilt $tilt"
            }
        }
        // Step sizes: don't overwrite a field the user is editing.
        if (!panStepInput.hasFocus() && status.has("ps")) panStepInput.setText(fmt("%.1f", status.optDouble("ps")))
        if (!tiltStepInput.hasFocus() && status.has("ts")) tiltStepInput.setText(fmt("%.1f", status.optDouble("ts")))
        status.optJSONArray("lim")?.let { lim ->
            limitsText.text = String.format(
                Locale.US, "Pan limits: min %.0f° / max %.0f°\nTilt limits: min %.0f° / max %.0f°",
                lim.optDouble(0), lim.optDouble(1), lim.optDouble(2), lim.optDouble(3)
            )
        }

        apName = status.optString("apSsid", apName)
        val homeName = status.optString("staSsid", "")
        if (!wifiSwitching) {
            val ap = status.optInt("ap", 1) == 1
            val wifiOn = status.optInt("wifi", 1) == 1
            wifiSwitch.isChecked = !ap
            wifiOnSwitch.isChecked = wifiOn
            wifiNote.text = when {
                !wifiOn -> "WiFi is off - Bluetooth only"
                status.optInt("fb") == 1 -> "Home WiFi \"$homeName\" not found - running as access point"
                ap -> "Hosting \"$apName\""
                else -> "Joined home WiFi \"$homeName\""
            }
        }
        // Fill the name fields once - after that, don't overwrite what the user types.
        if (!wifiFieldsLoaded && status.has("staSsid")) {
            staSsid.setText(homeName)
            apSsid.setText(apName)
            wifiFieldsLoaded = true
        }
        val res = status.optString("res", "")
        resText.text = (if (status.optInt("drv") == 1) "Driving...  " else "") + res
    }

    // ---------- UI helpers ----------

    private fun setupClick(id: Int, action: () -> Unit) {
        val b = findViewById<Button>(id)
        controlViews += b
        b.setOnClickListener { action() }
    }

    // Tap = one step. Hold = keep stepping while the BLE link is idle, so
    // steps never pile up in the queue after the finger is lifted.
    @SuppressLint("ClickableViewAccessibility")
    private fun setupHold(id: Int, cmd: String) {
        val b = findViewById<Button>(id)
        controlViews += b
        var repeater: Runnable? = null
        b.setOnTouchListener { v, e ->
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    v.isPressed = true
                    ble.send(cmd)
                    val r = object : Runnable {
                        override fun run() {
                            if (ble.isIdle) ble.send(cmd)
                            main.postDelayed(this, REPEAT_MS)
                        }
                    }
                    repeater = r
                    main.postDelayed(r, HOLD_DELAY_MS)
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    v.isPressed = false
                    repeater?.let { main.removeCallbacks(it) }
                    repeater = null
                }
            }
            true
        }
    }

    // One row per preset slot: "Preset N" + saved Az/Tilt, Go, Save.
    private fun buildPresetRows() {
        val rows = findViewById<android.widget.LinearLayout>(R.id.presetRows)
        val dp = resources.displayMetrics.density
        for (slot in 1..3) {
            val row = android.widget.LinearLayout(this).apply {
                orientation = android.widget.LinearLayout.HORIZONTAL
                gravity = android.view.Gravity.CENTER_VERTICAL
                setPadding(0, (6 * dp).toInt(), 0, 0)
            }
            val label = android.widget.LinearLayout(this).apply {
                orientation = android.widget.LinearLayout.VERTICAL
                layoutParams = android.widget.LinearLayout.LayoutParams(0, -2, 1f)
            }
            label.addView(TextView(this).apply {
                text = "Preset $slot"
                setTextColor(0xFFEEEEEE.toInt())
                textSize = 16f
            })
            val info = TextView(this).apply {
                text = "Az --, Tilt --"
                setTextColor(0xFF99AABB.toInt())
                textSize = 12f
            }
            label.addView(info)
            presetInfo += info
            row.addView(label)
            row.addView(presetButton("Go", 0xFF2D6CDF.toInt()) { ble.send("preset go $slot") })
            row.addView(presetButton("Save", 0xFF6B2FA0.toInt()) {
                confirm("Save the current position as Preset $slot?") { ble.send("preset save $slot") }
            })
            rows.addView(row)
        }
    }

    private fun presetButton(label: String, color: Int, action: () -> Unit) = Button(this).apply {
        text = label
        setTextColor(0xFFFFFFFF.toInt())
        backgroundTintList = android.content.res.ColorStateList.valueOf(color)
        setOnClickListener { action() }
        controlViews += this
    }

    private fun showSettings(show: Boolean) {
        val mainScreen = findViewById<View>(R.id.controls)
        val settingsScreen = findViewById<View>(R.id.settingsScreen)
        mainScreen.visibility = if (show) View.GONE else View.VISIBLE
        settingsScreen.visibility = if (show) View.VISIBLE else View.GONE
        findViewById<View>(R.id.btnBack).visibility = if (show) View.VISIBLE else View.GONE
        findViewById<View>(R.id.btnSettings).visibility = if (show) View.INVISIBLE else View.VISIBLE
        findViewById<TextView>(R.id.titleText).text = if (show) "Settings" else "Pan / Tilt Camera"
        hideKeyboard(settingsScreen)
        (mainScreen.parent.parent as? android.widget.ScrollView)?.scrollTo(0, 0)
    }

    @Deprecated("Deprecated in Java")
    override fun onBackPressed() {
        if (findViewById<View>(R.id.settingsScreen).visibility == View.VISIBLE) {
            showSettings(false)
        } else {
            @Suppress("DEPRECATION")
            super.onBackPressed()
        }
    }

    // WiFi commands restart the device: show that, and pause status updates of
    // the WiFi controls until it is back.
    private fun sendWifiCommand(cmd: String) {
        wifiSwitching = true
        wifiNote.text = "Device restarting..."
        ble.send(cmd)
        main.postDelayed({ wifiSwitching = false }, 8000)
    }

    private fun saveWifiConfig(kind: String, ssidField: EditText, passField: EditText, label: String) {
        val ssid = ssidField.text.toString().trim()
        val pass = passField.text.toString()
        when {
            ssid.isEmpty() -> toast("Enter a network name")
            ssid.contains('\n') || pass.contains('\n') -> toast("Line breaks are not allowed")
            pass.isNotEmpty() && pass.length < 8 -> toast("Password must be at least 8 characters")
            else -> confirm("Save $label settings? The device restarts to apply them.") {
                passField.setText("")
                hideKeyboard(passField)
                sendWifiCommand("wificfg $kind\n$ssid\n$pass")
            }
        }
    }

    private fun toast(text: String) = Toast.makeText(this, text, Toast.LENGTH_SHORT).show()

    private fun hideKeyboard(v: View) {
        (getSystemService(INPUT_METHOD_SERVICE) as InputMethodManager).hideSoftInputFromWindow(v.windowToken, 0)
    }

    private fun goAzimuth() {
        val t = azInput.text.toString().toFloatOrNull()
        if (t == null || t < 0f || t >= 360f) {
            Toast.makeText(this, "Enter an azimut from 0 to 359", Toast.LENGTH_SHORT).show()
            return
        }
        (getSystemService(INPUT_METHOD_SERVICE) as InputMethodManager)
            .hideSoftInputFromWindow(azInput.windowToken, 0)
        ble.send("goaz $t")
    }

    private fun confirm(message: String, onYes: () -> Unit) {
        AlertDialog.Builder(this)
            .setMessage(message)
            .setPositiveButton("Yes") { _, _ -> onYes() }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun setControlsEnabled(enabled: Boolean) {
        controlViews.forEach {
            it.isEnabled = enabled
            it.alpha = if (enabled) 1f else 0.4f
        }
    }

    private fun fmt(pattern: String, value: Double) = String.format(Locale.US, pattern, value)
}
