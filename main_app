import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

void main() {
  runApp(const FrogsApp());
}

class FrogsApp extends StatelessWidget {
  const FrogsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'F.R.O.G.S. Dashboard',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        brightness: Brightness.dark,
        scaffoldBackgroundColor: const Color(0xFF0D0D0D),
        colorScheme: const ColorScheme.dark(
          primary: Color(0xFFD4AF37),
          secondary: Color(0xFFFFD700),
          surface: Color(0xFF1A1A1A),
        ),
        appBarTheme: const AppBarTheme(
          backgroundColor: Color(0xFF111111),
          elevation: 0,
          centerTitle: true,
          titleTextStyle: TextStyle(
            color: Color(0xFFD4AF37),
            fontSize: 22,
            fontWeight: FontWeight.bold,
          ),
          iconTheme: IconThemeData(color: Color(0xFFD4AF37)),
        ),
        textTheme: const TextTheme(
          bodyLarge: TextStyle(color: Colors.white),
          bodyMedium: TextStyle(color: Colors.white70),
        ),
        useMaterial3: true,
      ),
      home: const DashboardScreen(),
    );
  }
}

class DashboardScreen extends StatefulWidget {
  const DashboardScreen({super.key});

  @override
  State<DashboardScreen> createState() => _DashboardScreenState();
}

class _DashboardScreenState extends State<DashboardScreen> {
  double tds = 145;
  double temperature = 72.5;
  double flowRate = 1.2;
  int battery = 82;

  bool pumpOn = false;
  bool connected = false;

  String bluetoothMessage = "Bluetooth scan not started";

  void togglePump() {
    setState(() {
      pumpOn = !pumpOn;
    });
  }

  Future<void> scanForBleDevices() async {
    setState(() {
      bluetoothMessage = "Checking Bluetooth permissions...";
    });

    await Permission.bluetoothScan.request();
    await Permission.bluetoothConnect.request();
    await Permission.locationWhenInUse.request();

    final isSupported = await FlutterBluePlus.isSupported;

    if (!isSupported) {
      setState(() {
        bluetoothMessage = "Bluetooth is not supported on this device.";
      });
      return;
    }

    final adapterState = await FlutterBluePlus.adapterState.first;

    if (adapterState != BluetoothAdapterState.on) {
      setState(() {
        bluetoothMessage = "Bluetooth is off. Turn Bluetooth on and try again.";
      });
      return;
    }

    setState(() {
      bluetoothMessage = "Scanning for nearby BLE devices...";
      connected = false;
    });

    final subscription = FlutterBluePlus.scanResults.listen((results) {
      for (ScanResult result in results) {
        final deviceName = result.advertisementData.advName.isNotEmpty
            ? result.advertisementData.advName
            : result.device.platformName;

        debugPrint("Found BLE device: $deviceName | ${result.device.remoteId}");

        if (deviceName.toUpperCase().contains("FROGS")) {
          setState(() {
            bluetoothMessage = "Found F.R.O.G.S. device: $deviceName";
            connected = true;
          });
        }
      }
    });

    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 8));

    await Future.delayed(const Duration(seconds: 8));

    await FlutterBluePlus.stopScan();
    await subscription.cancel();

    if (!connected) {
      setState(() {
        bluetoothMessage = "Scan complete. No F.R.O.G.S. device found.";
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    final width = MediaQuery.of(context).size.width;

    final crossAxisCount = width < 500
        ? 1
        : width < 900
            ? 2
            : 5;

    return Scaffold(
      appBar: AppBar(
        title: const Text("F.R.O.G.S. Water System"),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            children: [
              StatusHeader(
                connected: connected,
                warningText: bluetoothMessage,
              ),
              const SizedBox(height: 20),
              Expanded(
                child: GridView.count(
                  crossAxisCount: crossAxisCount,
                  crossAxisSpacing: 14,
                  mainAxisSpacing: 14,
                  childAspectRatio: 1.25,
                  children: [
                    InfoCard(
                      title: "TDS",
                      value: "${tds.toStringAsFixed(0)} ppm",
                      subtitle: "Water Purity",
                      icon: Icons.water_drop,
                    ),
                    InfoCard(
                      title: "Temperature",
                      value: "${temperature.toStringAsFixed(1)} °F",
                      subtitle: "Water Temperature",
                      icon: Icons.thermostat,
                    ),
                    InfoCard(
                      title: "Flow Rate",
                      value: "${flowRate.toStringAsFixed(1)} L/min",
                      subtitle: "Water Flow",
                      icon: Icons.speed,
                    ),
                    InfoCard(
                      title: "Battery",
                      value: "$battery%",
                      subtitle: "Power Status",
                      icon: Icons.battery_full,
                    ),
                    InfoCard(
                      title: "Pump",
                      value: pumpOn ? "ON" : "OFF",
                      subtitle: "Pump State",
                      icon: Icons.settings_input_component,
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: ElevatedButton.icon(
                      onPressed: scanForBleDevices,
                      icon: Icon(
                        connected
                            ? Icons.bluetooth_disabled
                            : Icons.bluetooth_connected,
                      ),
                      label: Text(
                        connected ? "Disconnect" : "Connect Device",
                      ),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: const Color(0xFFD4AF37),
                        foregroundColor: Colors.black,
                        padding: const EdgeInsets.symmetric(vertical: 16),
                        textStyle: const TextStyle(
                          fontWeight: FontWeight.bold,
                          fontSize: 16,
                        ),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 12),
                  Expanded(
                    child: OutlinedButton.icon(
                      onPressed: togglePump,
                      icon: Icon(
                        pumpOn ? Icons.stop_circle : Icons.play_circle_fill,
                      ),
                      label: Text(
                        pumpOn ? "Stop Pump" : "Start Pump",
                      ),
                      style: OutlinedButton.styleFrom(
                        foregroundColor: const Color(0xFFD4AF37),
                        side: const BorderSide(
                          color: Color(0xFFD4AF37),
                          width: 1.5,
                        ),
                        padding: const EdgeInsets.symmetric(vertical: 16),
                        textStyle: const TextStyle(
                          fontWeight: FontWeight.bold,
                          fontSize: 16,
                        ),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14),
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class StatusHeader extends StatelessWidget {
  final bool connected;
  final String warningText;

  const StatusHeader({
    super.key,
    required this.connected,
    required this.warningText,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(18),
      decoration: BoxDecoration(
        color: const Color(0xFF161616),
        borderRadius: BorderRadius.circular(18),
        border: Border.all(
          color: const Color(0xFFD4AF37),
          width: 1.4,
        ),
        boxShadow: const [
          BoxShadow(
            color: Colors.black54,
            blurRadius: 8,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          const Text(
            "System Status",
            style: TextStyle(
              color: Color(0xFFD4AF37),
              fontSize: 20,
              fontWeight: FontWeight.bold,
            ),
          ),
          const SizedBox(height: 12),
          Row(
            children: [
              Icon(
                connected ? Icons.check_circle : Icons.cancel,
                color: connected ? Colors.greenAccent : Colors.redAccent,
              ),
              const SizedBox(width: 8),
              Text(
                connected ? "Device Connected" : "Device Disconnected",
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 16,
                ),
              ),
            ],
          ),
          const SizedBox(height: 8),
          Row(
            children: [
              const Icon(
                Icons.warning_amber_rounded,
                color: Color(0xFFD4AF37),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  warningText,
                  style: const TextStyle(
                    color: Colors.white70,
                    fontSize: 15,
                  ),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class InfoCard extends StatelessWidget {
  final String title;
  final String value;
  final String subtitle;
  final IconData icon;

  const InfoCard({
    super.key,
    required this.title,
    required this.value,
    required this.subtitle,
    required this.icon,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: const Color(0xFF161616),
        borderRadius: BorderRadius.circular(18),
        border: Border.all(
          color: const Color(0xFFD4AF37),
          width: 1.2,
        ),
        boxShadow: const [
          BoxShadow(
            color: Colors.black45,
            blurRadius: 6,
            offset: Offset(0, 3),
          ),
        ],
      ),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(
            icon,
            color: const Color(0xFFD4AF37),
            size: 34,
          ),
          const SizedBox(height: 12),
          Text(
            title,
            style: const TextStyle(
              color: Color(0xFFD4AF37),
              fontSize: 18,
              fontWeight: FontWeight.bold,
            ),
          ),
          const SizedBox(height: 10),
          Text(
            value,
            textAlign: TextAlign.center,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 24,
              fontWeight: FontWeight.w700,
            ),
          ),
          const SizedBox(height: 6),
          Text(
            subtitle,
            textAlign: TextAlign.center,
            style: const TextStyle(
              color: Colors.white70,
              fontSize: 14,
            ),
          ),
        ],
      ),
    );
  }
}
