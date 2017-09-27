package com.example.s580l129.robot_lab3;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.content.Intent;
import android.os.Build;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import static java.nio.ByteOrder.LITTLE_ENDIAN;

public class MainActivity extends AppCompatActivity {

    // UUIDs for UAT service and associated characteristics.
    public UUID UART_UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    public UUID TX_UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

    // BTLE state
    public BluetoothAdapter adapter;
    public BluetoothGatt gatt;
    public BluetoothGattCharacteristic tx;
    private String statusString = "Initialized!";
    private String buttonString = "Connect";

    // Location Permissions
    private static final int PERMISSION_REQUEST_COARSE_LOCATION = 456;
    private static final int REQUEST_ENABLE_BT = 1;

    // Views
    private Button bluetoothButton;
    private TextView textStatus;
    private TextView textLeft;
    private TextView textRight;

    // Speed levels for the two motors
    final String ROBOT_NAME = "ABCSBOT";
    public int leftLevel = 128;
    public int rightLevel = 128;
    public int ctlByte = 0;
    public int lastMessage = 128 << 16 | 128 << 8;

    // Handler for mouse click on the send button.
    public void sendClick(View view) {
        // Determine what button initiated this
        if(view.getId() == R.id.F){
            //message = getForwardMessage();
            leftLevel = 176;
            rightLevel = 176;
        }
        else if(view.getId() == R.id.S){
            //message = getBackwardMessage();
            leftLevel = 128;
            rightLevel = 128;
        }
        else if(view.getId() == R.id.L){
            //message = getLeftMessage();
            leftLevel = 128;
            rightLevel = 160;
        }
        else if(view.getId() == R.id.R){
            //message = getRightMessage();
            leftLevel = 160;
            rightLevel = 128;
        }

        // Update levels display
        textLeft.setText(Integer.toString(leftLevel));
        textRight.setText(Integer.toString(rightLevel));
    }

    public void bluetoothClick(View view) {
        if (bluetoothButton.getText().equals("Connect")) {
            adapter.startLeScan(scanCallback);
            statusString = "Searching!";
            buttonString = "Stop";
        } else if (bluetoothButton.getText().equals("Stop")) {
            adapter.stopLeScan(scanCallback);
            statusString = "Disconnected!";
            buttonString = "Connect";
        } else if (bluetoothButton.getText().equals("Disconnect")) {
            gatt.disconnect();
            gatt.close();
            gatt = null;
            tx = null;
            statusString = "Disconnected!";
            buttonString = "Connect";
        }
    }

    public void toggleMusic(View view) {
        ctlByte = ctlByte ^ 1;
    }

    // Main BTLE device callback where much of the logic occurs.
    private BluetoothGattCallback callback = new BluetoothGattCallback() {
        // Called whenever the device connection state changes, i.e. from disconnected to connected.
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            super.onConnectionStateChange(gatt, status, newState);
            if (newState == BluetoothGatt.STATE_CONNECTED) {
                Log.i("BluetoothGattCallback", "Connected!");
                // Discover services.
                if (!gatt.discoverServices()) {
                    Log.i("BluetoothGattCallback", "Failed to start discovering services!");
                }
            }
            else if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                Log.i("BluetoothGattCallback", "Disconnected!");
                statusString = "Disconnected!";
                buttonString = "Connect";
            }
            else {
                Log.i("BluetoothGattCallback", "Connection state changed.  New state: " + newState);
            }
        }

        // Called when services have been discovered on the remote device.
        // It seems to be necessary to wait for this discovery to occur before
        // manipulating any services or characteristics.
        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            super.onServicesDiscovered(gatt, status);
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.i("BluetoothGattCallback", "Service discovery completed!");
                statusString = "Connected!";
                buttonString = "Disconnect";
            }
            else {
                Log.i("BluetoothGattCallback", "Service discovery failed with status: " + status);
            }
            // Save reference to each characteristic.
            tx = gatt.getService(UART_UUID).getCharacteristic(TX_UUID);
        }

        // Called when a remote characteristic changes (like the RX characteristic).
        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            super.onCharacteristicChanged(gatt, characteristic);
            Log.i("BluetoothGattCallback", "Received: " + characteristic.getStringValue(0));
        }
    };

    // BTLE device scanning callback.
    private BluetoothAdapter.LeScanCallback scanCallback = new BluetoothAdapter.LeScanCallback() {
        // Called when a device is found.
        @Override
        public void onLeScan(BluetoothDevice bluetoothDevice, int i, byte[] bytes) {
            Log.i("BluetoothGattCallback", "Found device: " + bluetoothDevice.getAddress());
            // Check if the device has the UART service.
            if (parseUUIDs(bytes).contains(UART_UUID) && bluetoothDevice.getName().equals(ROBOT_NAME)) {
                // Found a device, stop the scan.
                adapter.stopLeScan(scanCallback);
                Log.i("BluetoothGattCallback", "Found UART service!");
                        // Connect to the device.
                // Control flow will now go to the callback functions when BTLE events occur.
                gatt = bluetoothDevice.connectGatt(getApplicationContext(), false, callback);
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Get the TextViews used for display
        textStatus = (TextView) findViewById(R.id.textStatus);
        textLeft = (TextView) findViewById(R.id.textLeft);
        textRight = (TextView) findViewById(R.id.textRight);

        // Get the Buttons used for control
        bluetoothButton = (Button) findViewById(R.id.connectButton);
        View buttonLeft = findViewById(R.id.buttonLeft);
        View buttonRight = findViewById(R.id.buttonRight);

        adapter = BluetoothAdapter.getDefaultAdapter();

        // Handler for status bar
        final Handler myHandler = new Handler();
        class MyRunnableStatus implements Runnable {
            private Handler handler;
            private MyRunnableStatus(Handler handler) {
                this.handler = handler;
            }
            @Override
            public void run() {
                this.handler.postDelayed(this, 500);
                textStatus.setText(statusString);
            }
        }
        class MyRunnableButtonText implements Runnable {
            private Handler handler;
            private MyRunnableButtonText(Handler handler) {
                this.handler = handler;
            }
            @Override
            public void run() {
                this.handler.postDelayed(this, 500);
                bluetoothButton.setText(buttonString);
            }
        }
        class MyRunnableBluetooth implements Runnable {
            private Handler handler;
            private MyRunnableBluetooth(Handler handler) {
                this.handler = handler;
            }
            @Override
            public void run() {
                this.handler.postDelayed(this, 100);

                int newMessage = leftLevel << 16 | rightLevel << 8 | ctlByte;

                if (tx != null && newMessage != lastMessage) {
                    byte[] message = ByteBuffer.allocate(4).putInt(newMessage).order(LITTLE_ENDIAN).array();

                    tx.setValue(message);
                    if (gatt.writeCharacteristic(tx)) {
                        Log.i("BluetoothThread", Integer.toString(leftLevel) + " " + Integer.toString(rightLevel) + " " + Integer.toString(ctlByte));
                        lastMessage = newMessage;
                    } else {
                        Log.i("BluetoothThread", "Could not write to TX");
                    }
                }
            }
        }
        myHandler.post(new MyRunnableStatus(myHandler));
        myHandler.post(new MyRunnableBluetooth(myHandler));
        myHandler.post(new MyRunnableButtonText(myHandler));

        buttonLeft.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent e) {
                float val = e.getY();
                float min = v.getTop();
                float max = v.getBottom();

                val = val < min ? min:val;
                val = val > max ? max:val;
                leftLevel = (int)(255 * (max-val)/(max-min));

                textLeft.setText(Integer.toString(leftLevel));
                textRight.setText(Integer.toString(rightLevel));

                return true;
            }
        });

        buttonRight.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent e) {
                float val = e.getY();
                float min = v.getTop();
                float max = v.getBottom();

                val = val < min ? min:val;
                val = val > max ? max:val;
                rightLevel = (int)(255 * (max-val)/(max-min));

                textLeft.setText(Integer.toString(leftLevel));
                textRight.setText(Integer.toString(rightLevel));

                return true;
            }
        });

        if (!adapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            requestPermissions(new String[]{Manifest.permission.ACCESS_COARSE_LOCATION}, PERMISSION_REQUEST_COARSE_LOCATION);
        }
    }


    // OnStop, called right before the activity loses foreground focus.  Close the BTLE connection.
    @Override
    protected void onStop() {
        super.onStop();
        if (gatt != null) {
            // For better reliability be careful to disconnect and close the connection.
            gatt.disconnect();
            gatt.close();
            gatt = null;
            tx = null;
        }
    }


    // Filtering by custom UUID is broken in Android 4.3 and 4.4, see:
    //   http://stackoverflow.com/questions/18019161/startlescan-with-128-bit-uuids-doesnt-work-on-native-android-ble-implementation?noredirect=1#comment27879874_18019161
    // This is a workaround function from the SO thread to manually parse advertisement data.
    private List<UUID> parseUUIDs(final byte[] advertisedData) {
        List<UUID> uuids = new ArrayList<>();

        int offset = 0;
        while (offset < (advertisedData.length - 2)) {
            int len = advertisedData[offset++];
            if (len == 0)
                break;

            int type = advertisedData[offset++];
            switch (type) {
                case 0x02: // Partial list of 16-bit UUIDs
                case 0x03: // Complete list of 16-bit UUIDs
                    while (len > 1) {
                        int uuid16 = advertisedData[offset++];
                        uuid16 += (advertisedData[offset++] << 8);
                        len -= 2;
                        uuids.add(UUID.fromString(String.format("%08x-0000-1000-8000-00805f9b34fb", uuid16)));
                    }
                    break;
                case 0x06:// Partial list of 128-bit UUIDs
                case 0x07:// Complete list of 128-bit UUIDs
                    // Loop through the advertised 128-bit UUID's.
                    while (len >= 16) {
                        try {
                            // Wrap the advertised bits and order them.
                            ByteBuffer buffer = ByteBuffer.wrap(advertisedData, offset++, 16).order(LITTLE_ENDIAN);
                            long mostSignificantBit = buffer.getLong();
                            long leastSignificantBit = buffer.getLong();
                            uuids.add(new UUID(leastSignificantBit,
                                    mostSignificantBit));
                        } catch (IndexOutOfBoundsException e) {
                            // Defensive programming.
                            //Log.e(LOG_TAG, e.toString());
                        } finally {
                            // Move the offset to read the next uuid.
                            offset += 15;
                            len -= 16;
                        }
                    }
                    break;
                default:
                    offset += (len - 1);
                    break;
            }
        }
        return uuids;
    }
}
