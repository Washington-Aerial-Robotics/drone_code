import 'package:flutter/material.dart';

// Purpose: 
//  Standalone widget showing:
//    1. IP field
//    2. Port field
//    3. Connect/Disconnect Button
// Basically a connection settings box
class ConnectPanel extends StatelessWidget {
  const ConnectPanel({
    super.key,
    required this.ipCtrl,
    required this.portCtrl,
    required this.connected,
    required this.connecting,
    required this.onConnect,
    required this.onDisconnect,
  });

  final TextEditingController ipCtrl;
  final TextEditingController portCtrl;
  final bool connected;
  final bool connecting;
  final VoidCallback onConnect;
  final VoidCallback onDisconnect;

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 2,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Text('Drone IP:', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            TextField(
              controller: ipCtrl,
              decoration: const InputDecoration(
                hintText: 'e.g., 172.20.10.7 or 192.168.4.1',
                border: OutlineInputBorder(),
                isDense: true,
              ),
              enabled: !connected && !connecting,
            ),
            const SizedBox(height: 12),
            Text('Port:', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            TextField(
              controller: portCtrl,
              keyboardType: TextInputType.number,
              decoration: const InputDecoration(
                hintText: '1234 (or 23 if using telnet port)',
                border: OutlineInputBorder(),
                isDense: true,
              ),
              enabled: !connected && !connecting,
            ),
            const SizedBox(height: 16),
            SizedBox(
              height: 56,
              child: ElevatedButton(
                onPressed: connected
                    ? onDisconnect
                    : (connecting ? null : onConnect),
                child: Text(
                  connected
                      ? 'Disconnect'
                      : (connecting ? 'Connecting…' : 'Connect to drone'),
                  style: const TextStyle(fontSize: 18),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
